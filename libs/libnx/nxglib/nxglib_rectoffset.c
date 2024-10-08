/****************************************************************************
 * libs/libnx/nxglib/nxglib_rectoffset.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_rectoffset
 *
 * Description:
 *   Offset the rectangle position by the specified dx, dy values.
 *
 ****************************************************************************/

void nxgl_rectoffset(FAR struct nxgl_rect_s *dest,
                     FAR const struct nxgl_rect_s *src,
                     nxgl_coord_t dx, nxgl_coord_t dy)
{
  dest->pt1.x = src->pt1.x + dx;
  dest->pt1.y = src->pt1.y + dy;
  dest->pt2.x = src->pt2.x + dx;
  dest->pt2.y = src->pt2.y + dy;
}
