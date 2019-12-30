## This Source is M5StickV MaixPy
# import network, socket, time, sensor, image,lcd
import sensor, image, time,lcd,machine,utime,array,math,os
import KPU as kpu
from Maix import GPIO
from fpioa_manager import fm, board_info
from machine import UART

#M5StickV Camera Start
clock = time.clock()
lcd.init()
lcd.rotation(2)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.run(1)
sensor.skip_frames(time = 2000)

#M5StickV GPIO_UART
fm.register(35, fm.fpioa.UART2_TX, force=True)
fm.register(34, fm.fpioa.UART2_RX, force=True)
uart_Port = UART(UART.UART2, 115200,8,0,0, timeout=1000, read_buf_len= 4096)

#M5StickV ButtonA

fm.register(board_info.BUTTON_A, fm.fpioa.GPIO1)
but_a=GPIO(GPIO.GPIO1, GPIO.IN, GPIO.PULL_UP) #PULL_UP is required here!

fm.register(board_info.BUTTON_B, fm.fpioa.GPIO2)
but_b = GPIO(GPIO.GPIO2, GPIO.IN, GPIO.PULL_UP) #PULL_UP is required here!

task = kpu.load(0x300000)
anchor = (1.889, 2.5245, 2.9465, 3.94056, 3.99987, 5.3658, 5.155437, 6.92275, 6.718375, 9.01025)
a = kpu.init_yolo2(task, 0.5, 0.3, 5, anchor)

while(True):
    clock.tick()
    img = sensor.snapshot()
    code = kpu.run_yolo2(task, img)
    lcd.display(img)
    if code:
        for i in code:
            print(i)
            a = img.draw_rectangle(i.rect())
            b = lcd.display(img)
            img_buf = img.compress(quality=70)
            img_size1 = (img.size()& 0xFF0000)>>16
            img_size2 = (img.size()& 0x00FF00)>>8
            img_size3 = (img.size()& 0x0000FF)>>0
            data_packet = bytearray([0xFF,0xD8,0xEA,0x01,img_size1,img_size2,img_size3,0x00,0x00,0x00])
            uart_Port.write(data_packet)
            uart_Port.write(img_buf)
            time.sleep(1.0)

a = kpu.deinit(task)

#   Send UART End
uart_Port.deinit()
del uart_Port
print("finish")
