#include "silkit_can_backend.h"

#include <silkit/SilKit.hpp>
#include <silkit/services/can/all.hpp>
#include <silkit/services/orchestration/all.hpp>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <deque>
#include <future>
#include <chrono>
#include <thread>
#include <memory>
#include <mutex>
#include <string>


namespace
{

using SilKitCanController =
    SilKit::Services::Can::ICanController;

using ParticipantState =
    SilKit::Services::Orchestration::ParticipantState;

using OperationMode =
    SilKit::Services::Orchestration::OperationMode;


struct QueuedCanFrame
{
    CanFrame frame{};
    uint64_t rx_time_ns = 0U;
};


struct SilKitCanContext
{
    std::unique_ptr<SilKit::IParticipant> participant;

    SilKitCanController *controller = nullptr;

    SilKit::Services::Orchestration::ILifecycleService *lifecycle = nullptr;

    std::future<ParticipantState> lifecycle_future;

    SilKit::Util::HandlerId frame_handler_id{};

    std::mutex rx_mutex;

    std::deque<QueuedCanFrame> rx_queue;

    std::atomic<bool> ready{false};

    uint32_t bitrate = 1000000U;
};


CanBackendResult silkit_send(
    void *context_ptr,
    const CanFrame *frame
)
{
    if (
        context_ptr == nullptr ||
        frame == nullptr
    )
    {
        return CAN_BACKEND_ERROR;
    }

    auto *context =
        static_cast<SilKitCanContext *>(context_ptr);

    if (!context->ready.load())
    {
        return CAN_BACKEND_WOULD_BLOCK;
    }

    if (
        frame->id > CAN_STANDARD_ID_MAX ||
        frame->dlc > CAN_CLASSIC_MAX_DATA_BYTES
    )
    {
        return CAN_BACKEND_ERROR;
    }

    try
    {
        SilKit::Services::Can::CanFrame silkit_frame{};

        silkit_frame.canId = frame->id;

        /*
         * No FDF or BRS flags:
         * this is Classical CAN, not CAN FD.
         */
        silkit_frame.flags = 0;

        silkit_frame.dlc = frame->dlc;

        silkit_frame.dataField =
    SilKit::Util::Span<const uint8_t>(
        frame->data,
        frame->dlc
    );

        context->controller->SendFrame(
            silkit_frame
        );

        return CAN_BACKEND_OK;
    }
    catch (...)
    {
        return CAN_BACKEND_ERROR;
    }
}


CanBackendResult silkit_receive(
    void *context_ptr,
    CanFrame *frame
)
{
    if (
        context_ptr == nullptr ||
        frame == nullptr
    )
    {
        return CAN_BACKEND_ERROR;
    }

    auto *context =
        static_cast<SilKitCanContext *>(context_ptr);

    std::lock_guard<std::mutex> lock(
        context->rx_mutex
    );

    if (context->rx_queue.empty())
    {
        return CAN_BACKEND_WOULD_BLOCK;
    }

    *frame =
        context->rx_queue.front().frame;

    context->rx_queue.pop_front();

    return CAN_BACKEND_OK;
}


void silkit_close(
    void *context_ptr
)
{
    if (context_ptr == nullptr)
    {
        return;
    }

    auto *context =
        static_cast<SilKitCanContext *>(context_ptr);

    try
    {
        if (context->controller != nullptr)
        {
            context->controller->RemoveFrameHandler(
                context->frame_handler_id
            );

            if (context->ready.load())
            {
                context->controller->Stop();
            }
        }

        if (
            context->lifecycle != nullptr &&
            context->lifecycle->State() == ParticipantState::Running
        )
        {
            context->lifecycle->Stop(
                "CAN backend closed"
            );
        }
    }
    catch (...)
    {
        /* Closing should not throw into C code. */
    }

    delete context;
}

} /* namespace */


extern "C"
bool silkit_can_backend_create(
    CanBackend *backend,
    const SilKitCanBackendConfig *config
)
{
    if (
        backend == nullptr ||
        config == nullptr ||
        config->participant_name == nullptr ||
        config->controller_name == nullptr ||
        config->network_name == nullptr ||
        config->registry_uri == nullptr ||
        config->bitrate == 0U
    )
    {
        return false;
    }

    auto *context =
        new SilKitCanContext{};

    try
    {
        context->bitrate =
            config->bitrate;

        auto participant_configuration =
            SilKit::Config::ParticipantConfigurationFromString(
                ""
            );

        context->participant =
            SilKit::CreateParticipant(
                participant_configuration,
                config->participant_name,
                config->registry_uri
            );

        context->controller =
            context->participant->CreateCanController(
                config->controller_name,
                config->network_name
            );


        /*
         * Receive callback.
         *
         * SIL Kit invokes this asynchronously.
         * Frames are therefore copied into our own RX queue.
         */
        context->frame_handler_id =
            context->controller->AddFrameHandler(
                [context](
                    SilKitCanController *,
                    const SilKit::Services::Can::CanFrameEvent &event
                )
                {
                    const auto &incoming =
                        event.frame;

                    /*
                     * Our robot uses only Classical CAN 2.0A.
                     */
                    if (
                        incoming.canId >
                            CAN_STANDARD_ID_MAX ||
                        incoming.dataField.size() >
                            CAN_CLASSIC_MAX_DATA_BYTES
                    )
                    {
                        return;
                    }

                    CanFrame frame{};

                    frame.id =
                        static_cast<uint16_t>(
                            incoming.canId
                        );

                    frame.dlc =
                        static_cast<uint8_t>(
                            incoming.dataField.size()
                        );

                    std::copy(
                        incoming.dataField.begin(),
                        incoming.dataField.end(),
                        frame.data
                    );

                    const uint64_t rx_time_ns =
                        static_cast<uint64_t>(
                            std::chrono::duration_cast<
                                std::chrono::nanoseconds
                            >(
                                std::chrono::steady_clock::now()
                                    .time_since_epoch()
                            ).count()
                        );

                    std::lock_guard<std::mutex> lock(
                        context->rx_mutex
                    );

                    context->rx_queue.push_back(
                        QueuedCanFrame{
                            frame,
                            rx_time_ns
                        }
                    );
                }
            );


        /*
         * Use an autonomous lifecycle for the PC simulation backend.
         */
        context->lifecycle =
            context->participant->CreateLifecycleService(
                {OperationMode::Autonomous}
            );


        /*
         * SIL Kit recommends configuring and starting
         * the CAN controller from CommunicationReady.
         */
        context->lifecycle
            ->SetCommunicationReadyHandler(
                [context]()
                {
                    /*
                     * First argument is the Classical CAN rate.
                     *
                     * FD and XL rates are irrelevant because
                     * this project sends only Classical CAN.
                     */
                    context->controller->SetBaudRate(
                        context->bitrate,
                        context->bitrate,
                        context->bitrate
                    );

                    context->controller->Start();

                    context->ready.store(
                        true
                    );
                }
            );


        context->lifecycle_future =
            context->lifecycle->StartLifecycle();


        backend->context =
            context;

        backend->send =
            silkit_send;

        backend->receive =
            silkit_receive;

        backend->close =
            silkit_close;

        return true;
    }
    catch (...)
    {
        delete context;

        return false;
    }
}

extern "C"
bool silkit_can_backend_wait_ready(
    CanBackend *backend,
    uint32_t timeout_ms
)
{
    if (
        backend == nullptr ||
        backend->context == nullptr
    )
    {
        return false;
    }

    auto *context =
        static_cast<SilKitCanContext *>(backend->context);

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);

    while (
        std::chrono::steady_clock::now() < deadline
    )
    {
        if (context->ready.load())
        {
            return true;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1)
        );
    }

    return context->ready.load();
}


extern "C"
bool silkit_can_backend_receive_timestamped(
    CanBackend *backend,
    CanFrame *frame,
    uint64_t *rx_time_ns
)
{
    if (
        backend == nullptr ||
        backend->context == nullptr ||
        frame == nullptr ||
        rx_time_ns == nullptr
    )
    {
        return false;
    }

    auto *context =
        static_cast<SilKitCanContext *>(
            backend->context
        );

    std::lock_guard<std::mutex> lock(
        context->rx_mutex
    );

    if (context->rx_queue.empty())
    {
        return false;
    }

    const QueuedCanFrame queued =
        context->rx_queue.front();

    context->rx_queue.pop_front();

    *frame = queued.frame;
    *rx_time_ns = queued.rx_time_ns;

    return true;
}
