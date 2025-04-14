# ELEC 391 Course Project: Cascaded PID Control of a Two-Wheel Inverted Pendulum Robot

Submission Date: April 13, 2025 \
Mazda Farrahi, Qinnan Zhou &copy; 2025

**DISCLAIMER: As per UBC's Academic Honesty and Integrity policy, any usage of the code and/or resources within this public repository must be referenced by students. A violation of the aforementioned policy may result in charges of academic misconduct.**

## Quick Links:
**[BOM LIST](#bom-list)**

**[YOUTUBE LINK: VIDEO DEMO](https://www.youtube.com/watch?v=I9QuAMcz9hc)** 

**[PROJECT REPORT PDF](/Final_Report.pdf)** 



---

**INFO:** This project is for ELEC 391, the project course for third-year Electrical Engineering at UBC. This repository includes all the code written for this project, including MATLAB simulation code and Simulink files for the controls portion. 

The overall repository includes:
  - [Directory] [ESP32 Camera Access Point](/camera/)
  - [Directory] [MATLAB & Simulink Controls](/controls/)
  - [Directory] [Python GUI](/gui/)
  - [File] [Main Robot Code](/robot.ino)

## GUI
To launch the GUI, run [`dashboard.py`](/gui/dashboard.py) 

Required Python Packages:
- `cv2`
- `numpy`
- `PIL`
- `customtkinter`
- `tkinter`
- `pyserial`
- `bleak`
- `pyo`

## BOM List
| Item              | Price / Item | URL |
| :---------------- | ------: | ----: |
| ESP32-CAM        |   $22.90   | [Link](https://www.amazon.ca/CANADUINO-ESP32-CAM-ESP32-CAM-MB-Programming-Adapter/dp/B08XYLSH15/?_encoding=UTF8&pd_rd_w=R7Yr4&content-id=amzn1.sym.058704f3-b5a4-43fe-a9c9-166bf808d15b%3Aamzn1.symc.a68f4ca3-28dc-4388-a2cf-24672c480d8f&pf_rd_p=058704f3-b5a4-43fe-a9c9-166bf808d15b&pf_rd_r=SXB5S7CM9RN67M7ZKMC5&pd_rd_wg=MAHyZ&pd_rd_r=afa7088d-2d52-40ec-8f25-e2c39c2a88b5&ref_=pd_hp_d_atf_ci_mcx_mr_ca_hp_atf_d) |
| MG90S 9G Servo   |   $3.60  | [Link](https://www.amazon.ca/Miuzei-MG90S-Servo-Helicopter-Arduino/dp/B0BWJ26PX2/ref=pd_ci_mcx_mh_mcx_views_0_title?pd_rd_w=mR5Li&content-id=amzn1.sym.132f1a2b-de6d-4eb4-9982-d6fe080f8827%3Aamzn1.symc.40e6a10e-cbc4-4fa5-81e3-4435ff64d03b&pf_rd_p=132f1a2b-de6d-4eb4-9982-d6fe080f8827&pf_rd_r=9ST8C5HQ8XVH9VPJM6T2&pd_rd_wg=nlI9a&pd_rd_r=b5d6fd8c-e035-49f5-b521-f04291949884&pd_rd_i=B0BWJ26PX2&th=1) |
| (2x) DC-DC Buck Boost Voltage Converter   |  $3.20  | [Link](https://www.amazon.ca/XLX-XL6009-DC-DC-Converter-Power-Adjustable-Step-Down/dp/B08PP1N8G5/ref=sr_1_3?crid=2E7Z8CXQD21TS&dib=eyJ2IjoiMSJ9.Jbiwk6uMohth0GZpmb1dBHf1l-9HILZoFh-9yp4a-a4h42um6rH4aSVaGFZ6jRp-71Z9YKC0Dh0Teti9ezQIJ7l6_8M4d8tBHToK5HJQXrt5zQ9y6U55RfBivhzyaSRQZ_ZZRjwturfJLFw29Q9-57UCNTeG0m4YGicwimgIFtph5v0aADox_V-AzhYavk4Uppa8scxW9mx8D49fTEu0xV9KgDf1CnN3ce1iHuFjEW3hzzIQ5VgTfAkyoCfrUjHXuw_pD7nXBS1-S6FSp0P5oGwIVCLGCJnI8ny_azoIeR8.6AQDGKEaZ19FHcHszn3ULWZmbIejtfOM3Gz9RbLCeT4&dib_tag=se&keywords=buck+boost+converter&qid=1744616227&s=industrial&sprefix=buck+boost+converte%2Cindustrial%2C139&sr=1-3) |
| MPU-6050 MPU6050 Sensor |  $5.33   | [Link](https://www.amazon.ca/Wishiot-MPU-6050-Accelerometer-Gyroscope-Converter/dp/B0C3LG1Y6W/ref=sr_1_2_sspa?crid=1K6ZOTM6HQG14&dib=eyJ2IjoiMSJ9.BiBAol8t1f6nzfhLZASEDJY91HHVILzJ-Ssb-pUbuCw1nOohfb5Jf3f4E15bBS1yWFtxEwyPrdnT-siRrX_PYck1VN2s_Y1W4Ll8X43w_KwsF7IBv4AnqHYGVJ4xM9qqaBSiGpUU8ObW-S0XPZJK4H5P6VkuxhGh8qfK7TAwr0MJnnse_y94i9d-25HuCuyV5oSpMj6Iw8mBeyDd4Cp02WvTljn6U04P6jk-7GLHUclbW_dKTHq9hGvhjTEd2YreJnDGoZzVV8KxTRIRl41KC4j1b8nLGqyzW9g55vouzqs.qWi47HsCkk66YzFeyUy-MKNdsfLfCFYawNFxkD9I9CQ&dib_tag=se&keywords=MPU6050&qid=1744616248&s=industrial&sprefix=mpu60%2Cindustrial%2C301&sr=1-2-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1) |
| ELEGOO UNO R3 Board ATmega328P |  $21.99  | [Link](https://www.amazon.ca/Elegoo-Board-ATmega328P-ATMEGA16U2-Arduino/dp/B01EWOE0UU/ref=sr_1_1_sspa?crid=2T76GK7OMQKJQ&dib=eyJ2IjoiMSJ9.yTJWcx_WaSx7yq77bFuJ5doYqH69SjpPiXEyGgDAetYYWD3z1hYt2JEd9eCOpF4Bc9Ehu48-Y0V1hb-Ae2dRozQ05QqV2MSXpucNvG64HvYAdUdPo4ZKQ7dh3BfNefncFqaY2qC9YBroL8oNQybqbEuDSjW6LtKz7qHC4mtRe0E1YhtZaIP6_ACLEE12ywbapKjT8N1YrnUupPxtGSwcXb0R8IhhGKa83PtUSBq07b9OVkHOYa-VPsryKIeFsUPq-Oi8tL7sPZ-zmnijBLpDGByQdeKcei5C7K115h1mlsQ.qON35wCj3W9wE-srnyXy4Rzb3cMDETmDAqDPIXilBxQ&dib_tag=se&keywords=arduino+uno&qid=1744616268&s=industrial&sprefix=arduino+uno%2Cindustrial%2C148&sr=1-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1) |
| **TOTAL** | **$60.22**| |