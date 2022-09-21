#ifndef CONST
#define CONST

#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 720

enum COLOR{
    BLUE = 1,
    RED = 2,
    OTHER_COLOR = 3
};

enum ArmorType{
    SMALL = 0,
    BIG = 1,
    RUNE_ARMOR = 2
};

enum ArmorState{
    LOST = 0,       // 丢失目标
    FIRST = 1,      // 第一次发现目标
    SHOOT = 2,      // 持续识别目标
    FINDING = 3     // 丢失目标但在寻找目标
};

enum Mode{
    NORMAL = 1,     // 普通模式
    RUNE = 2,       // 大符模式
    NORMAL_RUNE = 3 // 小符模式
};

#endif