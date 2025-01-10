# arm_hand_deployment

目前需要支持：

- franka arm + franka gripper
- franka arm + allegro hand
- franka arm + 因时灵巧手


所以最好是把这个代码库设计成可拓展的，后续可以更新各种组件。



## TODO

- [ ] franka arm + franka gripper. 因为franka arm + allegro hand老代码库已经支持了，这是需要的第一个新功能。而且它仅依赖deoxys。话说,deoxys已经是server-client架构了，不过deoxys要system-wide安装傻逼3.13.0版本的protobuf，而且为了接口的统一性，就糊一层吧。