#ifndef A6EC_IDENTITY_H
#define A6EC_IDENTITY_H


#include <stdint.h>


/* ============================================================================
 * A6-EC ETHERCAT IDENTITY
 * ============================================================================
 *
 * IMPORTANT:
 *
 * These values are the identity reported by the CURRENT KickCAT simulator:
 *
 *      Slave name   : A6N_sAxis_V0.04
 *      Vendor ID    : 0x00400000
 *      Product Code : 0x00000715
 *
 * They are NOT yet confirmed as the identity of the real physical A6-EC.
 *
 * Before deploying to real hardware, replace/verify these values against
 * the real A6-EC ESI/SII information.
 * ============================================================================
 */

#define A6EC_EXPECTED_VENDOR_ID       ((uint32_t)0x00400000UL)

#define A6EC_EXPECTED_PRODUCT_CODE    ((uint32_t)0x00000715UL)


#endif /* A6EC_IDENTITY_H */