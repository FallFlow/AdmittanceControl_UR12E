# AdmittanceControl_UR12E

这是一个用于实现ur12e机械臂的导纳控制的代码库  
This is a repository used to implement admittance control for the ur12e.

pc和机械臂的通讯是通过**ur_rtde**库完成的，所以如果您需要使用我的代码，请先下载好相关的库和依赖  
The communication between the PC and the robotic arm is accomplished through the **ur_rtde** library. 
Therefore, if you need to use my code, please download the relevant libraries and dependencies first.

## 环境配置 Configuration Environment
关于**ur_rtde**库的使用请参考：  
For the use of the **ur_rtde** library, please refer to the following link  
https://sdurobotics.gitlab.io/ur_rtde/introduction/introduction.html

> 注意，ur_rtde是一个第三方库，开源链接是：  
> Note that ur_rtde is a third-party library. The open-source link is:  
> https://gitlab.com/sdurobotics/ur_rtde  
> 请和UR官方开源的RTDE库区分  
> Please separate it from the RTDE repository officially open-sourced by UR  
> 有关于UR官方RTDE协议的内容，可以参考  
> https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/
