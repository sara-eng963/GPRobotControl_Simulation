#include "../Guidance/virtual_ft_sensor.h"
#include "../../HandGuiding/wrench_frame.h"
#include "../../ControlCore/Config/robot_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    RobotConfig robot;
    robot_config_init_ur5(&robot);
    SensorConfig sensor = {0};
    const double h=sqrt(0.5);
    sensor.geometry_verified=true;
    sensor.flange_T_sensor.rotation_wxyz[0]=h;
    sensor.flange_T_sensor.rotation_wxyz[3]=h;
    sensor.flange_T_sensor.translation_m[0]=0.14;
    sensor.sensor_T_grip.rotation_wxyz[0]=1.0;
    robot.tool.T_F_TCP[0][3]=0.09;
    robot.tool.T_F_TCP[1][3]=-0.05;
    const FtSensorBias bias = {.valid=true};
    const double requested[6]={1.0,-2.0,0.8,0.1,-0.2,0.3};
    double measured[6]={0.2,-1.0,1.3,-0.5,0.7,0.4};
    for (size_t pass=0;pass<2;++pass) {
        if(pass==1) {measured[0]=-0.3; measured[4]=-0.7;}
        FtSensorSample sample={0};
        double recovered[6]={0};
        assert(gp_virtual_ft_sensor_sample(requested,&sensor,&robot,
                    measured,1234U,&sample));
        assert(wrench_frame_sensor_to_base_tcp(&sample,1244U,1000U,
                    &sensor,&bias,&robot,measured,true,recovered));
        for(size_t i=0;i<6;++i)
            assert(fabs(requested[i]-recovered[i])<1e-8);
    }
    sensor.geometry_verified=false;
    FtSensorSample sample={0};
    assert(!gp_virtual_ft_sensor_sample(requested,&sensor,&robot,
                measured,1400U,&sample));
    assert(!sample.valid);
    puts("PASS: virtual F/T base/TCP-to-sensor roundtrip, offsets and rotation");
    return 0;
}
