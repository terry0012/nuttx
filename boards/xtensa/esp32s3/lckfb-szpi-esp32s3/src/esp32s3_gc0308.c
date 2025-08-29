/****************************************************************************
 * drivers/video/sccb.c
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/


#include <debug.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/fs.h>
#include "esp32s3_i2c.h"
#include "esp32s3-szpi.h"
#include "syslog.h"

#define GC0308_I2C_ADDR 0x41
#define GC0308_PID_REG 0x00
#define GC0308_PID_VAL 0x9b
#define SCCB_FREQ (100 * 1000)

#define ESP32S3_GC0308_I2C_PORT 0

struct sccb_dev_s
{
  FAR struct i2c_master_s *i2c;   /* I2C 主设备接口 */
  uint8_t addr;                   /* SCCB 设备地址 (8位格式) */
  uint32_t freq;                  /* I2C 总线频率 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sccb_transfer
 *
 * Description:
 *   执行 SCCB/I2C 传输
 *
 ****************************************************************************/

static int sccb_transfer(FAR struct sccb_dev_s *dev, FAR struct i2c_msg_s *msgs,
                         int count)
{
  int ret;

  if (!dev || !dev->i2c || !msgs || count <= 0)
    {
      return -EINVAL;
    }

  /* 使用 NuttX 的 I2C 传输接口 */
  ret = I2C_TRANSFER(dev->i2c, msgs, count);
  if (ret < OK)
    {
      syslog(6, "ERROR: I2C_TRANSFER failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sccb_init
 ****************************************************************************/

static int sccb_init(FAR struct sccb_dev_s *dev, FAR struct i2c_master_s *i2c,
              uint8_t addr, uint32_t freq)
{
  if (!dev || !i2c)
    {
      return -EINVAL;
    }

  /* 初始化 SCCB 设备结构体 */
  dev->i2c = i2c;
  dev->addr = addr;
  dev->freq = (freq > 0) ? freq : 400000; /* 默认 400kHz */

  syslog(6,"SCCB initialized: addr=0x%02X, freq=%" PRIu32 " Hz\n",
          addr, dev->freq);

  return OK;
}

/****************************************************************************
 * Name: sccb_write_byte
 ****************************************************************************/

static int sccb_write_byte(FAR struct sccb_dev_s *dev, uint8_t reg_addr,
                    uint8_t data)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  int ret;

  if (!dev)
    {
      return -EINVAL;
    }

  /* 准备要发送的数据：寄存器地址 + 数据 */
  buffer[0] = reg_addr;
  buffer[1] = data;

  /* 设置 I2C 消息 */
  msg.frequency = dev->freq;
  msg.addr = dev->addr;
  msg.flags = 0; /* 写操作 */
  msg.buffer = buffer;
  msg.length = 2;

  /* 执行传输 */
  ret = sccb_transfer(dev, &msg, 1);
  if (ret < OK)
    {
      syslog(6,"ERROR: sccb_write_byte(0x%02X, 0x%02X) failed: %d\n",
             reg_addr, data, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sccb_read_byte
 ****************************************************************************/

static int sccb_read_byte(FAR struct sccb_dev_s *dev, uint8_t reg_addr,
                   uint8_t *data)
{
  struct i2c_msg_s msg;
  int ret;

  if (!dev || !data)
    {
      return -EINVAL;
    }

  /* Phase 1: 写入要读取的寄存器地址 */
  msg.frequency = dev->freq;
  msg.addr = dev->addr;
  msg.flags = 0; /* 写操作 */
  msg.buffer = &reg_addr;
  msg.length = 1;

  ret = I2C_TRANSFER(dev->i2c, &msg, 1);
  if (ret < OK) {
      syslog(6, "ERROR: Phase1 failed: %d\n", ret);
      return ret;
  }

  msg.flags = I2C_M_READ; /* 读操作，有START和STOP */
  msg.buffer = data;
  msg.length = 1;

  ret = I2C_TRANSFER(dev->i2c, &msg, 1);
  if (ret < OK) {
      syslog(6, "ERROR: Phase2 failed: %d\n", ret);
  }

  return ret;
}

/****************************************************************************
 * Name: sccb_write_bytes
 ****************************************************************************/

static int sccb_write_bytes(FAR struct sccb_dev_s *dev, uint8_t start_reg,
                     FAR const uint8_t *data, int len)
{
  FAR uint8_t *buffer;
  struct i2c_msg_s msg;
  int ret;

  if (!dev || !data || len <= 0)
    {
      return -EINVAL;
    }

  /* 分配缓冲区：寄存器地址 + 数据 */
  buffer = (FAR uint8_t *)kmm_malloc(len + 1);
  if (!buffer)
    {
      return -ENOMEM;
    }

  /* 准备数据 */
  buffer[0] = start_reg;
  memcpy(&buffer[1], data, len);

  /* 设置 I2C 消息 */
  msg.frequency = dev->freq;
  msg.addr = dev->addr;
  msg.flags = 0;
  msg.buffer = buffer;
  msg.length = len + 1;

  /* 执行传输 */
  ret = sccb_transfer(dev, &msg, 1);

  /* 释放缓冲区 */
  kmm_free(buffer);

  if (ret < OK)
    {
      syslog(6,"ERROR: sccb_write_bytes(0x%02X, len=%d) failed: %d\n",
             start_reg, len, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sccb_read_bytes
 ****************************************************************************/

static int sccb_read_bytes(FAR struct sccb_dev_s *dev, uint8_t start_reg,
                    uint8_t *data, int len)
{
  struct i2c_msg_s msgs[2];
  int ret;

  if (!dev || !data || len <= 0)
    {
      return -EINVAL;
    }

  /* Phase 1: 写入起始寄存器地址 */
  msgs[0].frequency = dev->freq;
  msgs[0].addr = dev->addr;
  msgs[0].flags = 0;
  msgs[0].buffer = &start_reg;
  msgs[0].length = 1;

  /* Phase 2: 读取多个数据字节 */
  msgs[1].frequency = dev->freq;
  msgs[1].addr = dev->addr;
  msgs[1].flags = I2C_M_READ;
  msgs[1].buffer = data;
  msgs[1].length = len;

  /* 执行两次传输 */
  ret = sccb_transfer(dev, msgs, 2);
  if (ret < OK)
    {
      syslog(6,"ERROR: sccb_read_bytes(0x%02X, len=%d) failed: %d\n",
             start_reg, len, ret);
    }

  return ret;
}

/* I2C设备扫描函数 */
static int i2c_scan_devices(FAR struct i2c_master_s *i2c, uint32_t freq)
{
  int found = 0;
  
  syslog(6, "Scanning I2C devices...\n");
  
  for (int addr = 0x08; addr <= 0x77; addr++) 
  {
    struct i2c_msg_s msg;
    uint8_t reg = 0x00; /* 尝试读取一个简单的寄存器 */
    
    msg.frequency = freq;
    msg.addr = addr;
    msg.flags = 0;
    msg.buffer = &reg;
    msg.length = 1;
    
    /* 尝试写操作来检测设备是否存在 */
    int ret = I2C_TRANSFER(i2c, &msg, 1);
    if (ret == OK) 
    {
      syslog(6, "Found device at address: 0x%02X\n", addr);
      found++;
    }
  }
  
  if (found == 0) 
  {
    syslog(6, "No I2C devices found!\n");
  }
  
  return found;
}

/* 在驱动初始化函数中 */
int gc0308_initialize()
{
  struct sccb_dev_s sccb_dev;
  int ret;

  struct file f;
  ssize_t n;

    ret = file_open(&f, "/dev/gpio2", O_RDWR);
  if (ret < 0)
    {
      syslog(6,"open C/S pin failed\n");
      return -ENODEV;
    }

  n = file_write(&f, "1", 1);

  usleep(20 * 1000);

  n = file_write(&f, "0", 1);
  usleep(20 * 1000);

  file_close(&f);
  if (n != 1)
    {
      syslog(6, "write C/S pin failed\n");
      return -EIO;
    }

  /* 1. 初始化 I2C 总线（内部有锁和引用计数，可安全多次调用） */
  FAR struct i2c_master_s *i2c_bus = esp32s3_i2cbus_initialize(ESP32S3_GC0308_I2C_PORT);
  if (i2c_bus == NULL)
  {
      return -ENODEV;
  }

  //i2c_scan_devices(i2c_bus, SCCB_FREQ);

  /* 2. 初始化 SCCB 设备 */
  ret = sccb_init(&sccb_dev, i2c_bus, GC0308_I2C_ADDR, SCCB_FREQ);
  if (ret != OK) 
  {
      return ret;
  }

//   ret = sccb_write_byte(&sccb_dev, 0xfe, 0x00);

  if (ret != OK) 
  {
      syslog(6,"ERROR: sccb_write_byte(0x%02X, 0x%02X) failed: %d\n",
             0xfe, 0x00, ret);
      return ret;
  }

  /* 3. 探测传感器 */
  uint8_t pid = 0;
  ret = sccb_read_byte(&sccb_dev, GC0308_PID_REG, &pid);

  syslog(6, ">>>>>>>>>>>>>>>>>>>GC0308 detected: PID=0x%02X\n", pid);

  if (ret == OK && pid == GC0308_PID_VAL) 
  {
      syslog(6,"GC0308 detected: PID=0x%02X\n", pid);
  }

  /* 4. 写入初始化序列 */
//   const uint8_t init_seq[] = {
//       0xFE, 0xF0, // 软件复位
//       // ... 更多寄存器配置
//   };
//   ret = sccb_write_bytes(&sccb_dev, 0x00, init_seq, sizeof(init_seq));
  
  return ret;
}