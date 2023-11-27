# 介绍
·`usr_main`的思路来自于学长，orz降维打击
# 使用方法
1. 在工程根目录创建`User`文件夹，将`usr_main.c`、`usr_main.h`及`相应文件夹`复制进`User`中

2. 在`main.c`文件中导入头文件`#include "usr_main.h`，并在代码区`while(1)`上方调用`usr_main();`

3. 此后所有的代码均在`usr_main.c`文件内编写
> (你可以把`usr_main.c`当作是`main.c`文件，`usr_main`函数作为`main`函数)
**尽量不对**工程目录的`Core`及`Drivers`文件夹所有子文件内容做修改

# 目的
直观、方便调试、方便代码浏览、方便移植...说不出词了XD