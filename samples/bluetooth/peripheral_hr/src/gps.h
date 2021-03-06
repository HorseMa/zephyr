/** @file
 *  @brief HRS Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef __cplusplus
extern "C" {
#endif

void gps_init(u8_t blsc);
void gps_notify(char *data);

#ifdef __cplusplus
}
#endif
