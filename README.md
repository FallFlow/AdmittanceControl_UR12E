# AdmittanceControl_UR12E

这是一个用于实现ur12e机械臂的导纳控制的代码库  
This is a repository used to implement admittance control for the ur12e.

上位机和机械臂的通讯是通过**ur_rtde**库完成的，所以如果您需要使用我的代码，请先下载好相关的库和依赖  
The communication between the upper computer and the robotic arm is accomplished through the **ur_rtde** library. 
Therefore, if you need to use my code, please download the relevant libraries and dependencies first.


## 环境配置 Configuration Environment

关于**ur_rtde**库的使用请参考：  
For the use of the **ur_rtde** library, please refer to the following link:  
https://sdurobotics.gitlab.io/ur_rtde/introduction/introduction.html

> 注意，ur_rtde是一个第三方库，开源链接是：  
> Note that ur_rtde is a third-party library. The open-source link is:  
> https://gitlab.com/sdurobotics/ur_rtde
> 
> 请和UR官方开源的RTDE库区分  
> Please separate it from the RTDE repository officially open-sourced by universal-robots.
> 
> 有关于UR官方RTDE协议的内容，可以参考
> For information regarding the official UR RTDE protocol, you can refer to the following link:  
> https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/


## 文件说明

admittance_control.py
> 这是导纳控制的程序，在上位机上运行它，机械臂将会根据末端受力情况进行导纳控制  
> This is an admittance control program. When run on the upper computer, the robotic arm will perform admittance control based on the forces detected at the end-effector.
> 我在编写代码的时候使用了虚拟环境（venv），也许你在使用代码的时候会出现一些库没安装的问题，不过我使用的都是很常见的库，直接pip安装就好
> When I developed this code, I used a virtual environment (venv). You might encounter some missing library errors when running it. However, the libraries I used are all very common ones; you can simply install them using **pip**

Filter.py
> 这是我自己写的滤波器，在admittance_control.py中会调用它  
> These are the filter functions I wrote, which are called by admittance_control.py

servoj_example.py
> 这是ur_rtde的开发者编写的例程，通过运行它来判断你是否成功安装了ur_rtde  
> This is an example program written by the developers of ur_rtde. Run it to verify whether you have successfully installed ur_rtde.

try.py
> 这是我自己编写的获取六维力传感器数据的脚本，不需要管它，或者你也可以运行它来判断你的UR机械臂是否拥有六维力传感器  
> This is a script I wrote to retrieve data from the six-axis force sensor. You can just ignore this file, or you can run it to check if your UR robotic arm is equipped with a six-axis force sensor.
