#ifndef IGH_MCAN_PORT_INTERNAL_H
#define IGH_MCAN_PORT_INTERNAL_H

#include "igh_mcan_port.h"

#define IGH_MCAN_RETURN_IF(condition, result) \
    do { \
        if (condition) { \
            return (result); \
        } \
    } while (0)
#define IGH_MCAN_CHECK_ARG(condition) \
    IGH_MCAN_RETURN_IF(!(condition), IGH_ECAN_INVALID_ARG)
#define IGH_MCAN_CHECK_READY(condition) \
    IGH_MCAN_RETURN_IF(!(condition), IGH_ECAN_NOT_READY)

#include "igh_mcan_pdo_internal.h"

#endif /* IGH_MCAN_PORT_INTERNAL_H */
