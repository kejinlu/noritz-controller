# 能率燃气热水器线控逆向工程

<img src="https://cdn.nlark.com/yuque/0/2022/jpeg/328998/1665892207477-41f3e260-5b2f-4b63-af00-7db736f00e69.jpeg?x-oss-process=image%2Fwatermark%2Ctype_d3F5LW1pY3JvaGVp%2Csize_26%2Ctext_5Y2i5YWL%2Ccolor_FFFFFF%2Cshadow_50%2Ct_80%2Cg_se%2Cx_10%2Cy_10%2Fresize%2Cw_900%2Climit_0" width="25%"/>


线控型号为 RC-7606M-A

缘由是能率的户外燃气热水器每次断电之后再次通电，会恢复到默认的关机状态，都需要人去按下开机按钮，比较麻烦。
另外也无法记录燃气热水器实时的状态。所以想通过对燃气热水的控制器进行逆向，然后通过esp32这类wifi芯片来将燃气热水器的相关信息接入Homeassistant。

硬件设计: https://oshwhub.com/kejinlu/noritz-controller

逆向过程详解：https://kejinlu.com/2022/10/noritz-controller/
