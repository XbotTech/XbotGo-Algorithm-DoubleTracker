
# 命名规则
通用命名规则 函数名、变量名、文件名应具有描述性，不要过度缩写

## 1、文件夹名
按功能模块命名，小写

## 2、类名
驼峰式命名，首字母大写 

## 3、文件名
文件命名规则同类名

## 4、函数名 & 变量名 
驼峰式命名 ，首字母小写 

## 5、枚举  
枚举类名属于类型名，按类命名，枚举值全大写加下划线

    enum class RtmpConnectResult {
		RTMP_CONNECT_RESULT_SUCCESS,
		RTMP_CONNECT_RESULT_FAILED,
		RTMP_CONNECT_RESULT_BREAK
	};

## 6、名字空间命名
命名空间全小写，基于项目名称和目录结构

    namespace XbotgoSDK
    {
    namespace wrapper
    {
    namespace api
    {
        
    }
    }
    }

## 7、大括号
函数体：分行写
函数内：不分行

    int RtmpProtocol::initRtmp(const std::string url)
    {
	    if(!RTMP_SetupURL(&rtmpInfo, const_cast<char*>(linkUrl.c_str()))) {
		    return -1;
	    }
	    return 0;
    }

## 8、头文件引用
指定好头文件路径，不使用相对路径

    #include<rtmp/connection.h>

## 9、禁止使用魔鬼数字
    
    return 0x11; 
    return 33;


## 10、* &靠近类型
    char* data；
    int& value；

## 11、空白
垂直空白越少越好

## 12、注释
doxygen规范

    /**
    * @brief 
    * @param  buffer           My Param doc
    * @param  len              My Param doc
    * @return int 
    */
    int platform_oled_write(uint8_t *buffer, uint16_t len);
