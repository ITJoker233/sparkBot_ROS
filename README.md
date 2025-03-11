# _
    SparkBot_

基于45coll版本上修改

嘉立创开源广场链接：[Sparkbot_45coll版本--无线充电、红外巡线 - 立创开源硬件平台](https://oshwhub.com/45coll/sparkbot-45coll)

Gitee链接：[sparkbot-45coll: sparkbot-45coll：sparkbot机器人项目，添加无线充电、小车巡线等功能，优化软硬件](https://gitee.com/coll45/sparkbot-45coll)

ESP IDF:v5.2.1

如果碰见依赖等问题无法编译问题，可尝试在终端输入执行 `python delete_dependencies_lock.py`

然后再删除 `build`目录**重新编译**

## Future Plan

1. [ ] 增加头部模块控制 - 利用陀螺仪操作？
2. [ ] 增加手柄控制？（暂无蓝牙手柄，且需要进行逆向工程）
3. [ ] 增加室内定位？待考虑方案
4. [ ] 增加上位机，通过绘画指定轨迹，进行轨迹运动

## Change Log

    2025/03/11 当前未验证，只是能编译过...  目前只有tank部分代码
