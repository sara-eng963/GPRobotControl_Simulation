/* Replaces SOEM's physical backend only. Keep ethercat_master, drive conversion,
 * PDO packing and CiA402 code in the build. Ideal position follower; no physics. */
#include "mock_backend.h"
#include "../../EtherCATComm/SOEM/soem_backend.h"
#include "../../ServoDrive/A6EC/a6ec_drive.h"
#include "../../ServoDrive/A6EC/a6ec_identity.h"
#include <string.h>
static uint8_t outputs[6][12], inputs[6][28];
static int8_t mode[6];
static uint16_t bus_state = 1;
static bool opened, fault, comm_failure;
static bool valid(int slave) { return slave >= 1 && slave <= 6; }
void soem_backend_write_u16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
void soem_backend_write_i32(uint8_t *p,int32_t v) { uint32_t u=(uint32_t)v; for(int i=0;i<4;i++) p[i]=(uint8_t)(u>>(8*i)); }
uint16_t soem_backend_read_u16(const uint8_t *p) { return (uint16_t)(p[0]|((uint16_t)p[1]<<8)); }
int32_t soem_backend_read_i32(const uint8_t *p) { uint32_t u=0; for(int i=0;i<4;i++)u|=(uint32_t)p[i]<<(8*i); int32_t v; memcpy(&v,&u,4);return v; }
void mock_backend_reset(void) { memset(outputs,0,sizeof(outputs));memset(inputs,0,sizeof(inputs));memset(mode,0,sizeof(mode));opened=fault=comm_failure=false;bus_state=1;for(int i=0;i<6;i++)soem_backend_write_u16(inputs[i]+2,0x40); }
void mock_backend_set_joint(unsigned j,double q) { if(j<6){int32_t u=a6ec_joint_rad_to_position_units(q);soem_backend_write_i32(inputs[j]+4,u);soem_backend_write_i32(outputs[j]+2,u);} }
double mock_backend_get_joint(unsigned j) { return j<6?a6ec_position_units_to_joint_rad(soem_backend_read_i32(inputs[j]+4)):0; }
void mock_backend_hold(void) { for(int i=0;i<6;i++)memcpy(outputs[i]+2,inputs[i]+4,4); }
void mock_backend_set_fault(bool a) { fault=a; }
void mock_backend_set_communication_failure(bool a) { comm_failure=a; }
bool mock_backend_ready(void) { if(!opened||fault||comm_failure||bus_state!=8)return false;for(int i=0;i<6;i++)if((soem_backend_read_u16(inputs[i]+2)&0x6f)!=0x27)return false;return true; }
bool soem_backend_open(const char *name) { opened=name!=NULL;return opened; }
void soem_backend_close(void) { opened=false; }
int soem_backend_scan(void) { if(opened)bus_state=2;return opened?6:0; }
const char *soem_backend_slave_name(int s) { return valid(s)?"PC mock A6-EC": "invalid"; }
bool soem_backend_slave_identity(int s,uint32_t*v,uint32_t*p,uint32_t*r,uint32_t*n) { if(!valid(s)||!v||!p||!r||!n)return false;*v=A6EC_EXPECTED_VENDOR_ID;*p=A6EC_EXPECTED_PRODUCT_CODE;*r=0x2ef8;*n=(uint32_t)s;return true; }
int soem_backend_map_pdos(void){if(opened)bus_state=4;return opened?240:0;}
int soem_backend_output_bytes(void){return 72;}
int soem_backend_input_bytes(void){return 168;}
int soem_backend_sdo_write(int s,uint16_t index,uint8_t sub,size_t size,const void*data){(void)sub;if(!opened||!valid(s)||!data||comm_failure)return 0;if(index==0x6060&&size==1){memcpy(&mode[s-1],data,1);return 1;}return 0;}
int soem_backend_sdo_read(int s,uint16_t index,uint8_t sub,size_t*size,void*data){(void)sub;if(!opened||!valid(s)||!size||!data||comm_failure)return 0;if((index==0x6060||index==0x6061)&&*size>=1){memcpy(data,&mode[s-1],1);*size=1;return 1;}if(index==0x603f&&*size>=2){uint16_t e=fault?1:0;memcpy(data,&e,2);*size=2;return 1;}return 0;}
bool soem_backend_has_error(void){return false;}
const char*soem_backend_pop_error_string(void){return "mock backend";}
bool soem_backend_configure_dc(void){return opened;}
bool soem_backend_slave_has_dc(int s){return valid(s);}
void soem_backend_sync0(int s,bool e,uint32_t c,int32_t sh){(void)s;(void)e;(void)c;(void)sh;}
uint16_t soem_backend_statecheck(int s,uint16_t requested,int t){(void)s;(void)requested;(void)t;return bus_state;}
int soem_backend_read_states(void){return opened?6:0;}
bool soem_backend_acknowledge_slave_error(int s){return valid(s);}
bool soem_backend_request_slave_operational(int s){if(!valid(s))return false;bus_state=8;return true;}
bool soem_backend_reconfigure_slave(int s){return valid(s)&&!comm_failure;}
bool soem_backend_recover_slave(int s){return valid(s)&&!comm_failure;}
uint16_t soem_backend_slave_state(int s){return valid(s)?bus_state:0;}
uint16_t soem_backend_slave_al_status_code(int s){(void)s;return 0;}
bool soem_backend_slave_is_lost(int s){return !valid(s)||comm_failure;}
const char*soem_backend_al_status_name(uint16_t s){(void)s;return "mock";}
void soem_backend_set_group_state(uint16_t s){bus_state=s;}
int soem_backend_write_group_state(void){return opened?1:0;}
const char*soem_backend_state_name(uint16_t s){switch(s){case 1:return "INIT";case 2:return "PRE-OP";case 4:return "SAFE-OP";case 8:return "OPERATIONAL";default:return "UNKNOWN";}}
int soem_backend_send_processdata(void){return opened&&!comm_failure?1:0;}
int soem_backend_receive_processdata(void){if(!opened||comm_failure)return 0;for(int i=0;i<6;i++){uint16_t cw=soem_backend_read_u16(outputs[i]);uint16_t sw=0x40;if(fault)sw=8;else if((cw&0xf)==0xf)sw=0x27;else if((cw&0x7)==0x7)sw=0x23;else if((cw&0x7)==0x6)sw=0x21;else if((cw&0x2)==0x2)sw=0x7;soem_backend_write_u16(inputs[i]+2,sw);if(sw==0x27)memcpy(inputs[i]+4,outputs[i]+2,4);}return 18;}
int soem_backend_expected_wkc(void){return 18;}
uint8_t*soem_backend_slave_outputs(int s){return valid(s)?outputs[s-1]:NULL;}
uint8_t*soem_backend_slave_inputs(int s){return valid(s)?inputs[s-1]:NULL;}
