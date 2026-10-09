#ifndef W25_ARTIFACT_CRC_H
#define W25_ARTIFACT_CRC_H
#include "../../../StateMachine/States/state_path_validation.h"
#include "w25q512jv_flash.h"
/* Match StateMachine/States/state_path_validation.c's field-wise CRC. */
static inline uint32_t w25_artifact_crc(const ValidatedTrajectory *a) {
    uint32_t crc = UINT32_C(0xFFFFFFFF);
#define C(x) do { crc=w25q512jv_crc32(crc,&(x),sizeof(x)); } while(0)
    C(a->program_id); C(a->source_revision); C(a->source_crc);
    C(a->sample_data_crc); C(a->sample_count); C(a->segment_count);
    C(a->sample_period_us); C(a->duration_s); C(a->path_length_m);
    for (uint16_t i=0; i<a->segment_count; ++i) {
        C(a->segments[i].first_sample); C(a->segments[i].sample_count);
        C(a->segments[i].segment_type);
    }
#undef C
    return ~crc;
}
#endif
