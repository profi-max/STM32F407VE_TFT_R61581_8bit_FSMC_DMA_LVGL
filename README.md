# STM32F407VE with 3.95-inch TFT R61581 8-bit parallel (FSMC DMA)

CubeIDE demo project.

LVGL benchmark shows weighted 91 FPS.

[![Watch the video](https://img.youtube.com/vi/ogDc5SFtDnM/hqdefault.jpg)](https://www.youtube.com/embed/ogDc5SFtDnM)

![board](/Pictures/board.jpg)

![board](/Pictures/stm32f407vet6.png)


## MCUFRIEND 3.95 TFT 480x320

![lcd](/Pictures/lcd_1.jpg)

## FSMC GPIO Configuration (LCD connection)
  
+  	FSMC_NE1(PD7) -> LCD_CS
+	FSMC_NWE(PD5) -> LCD_WR
+	FSMC_NOE(PD4) -> LCD_RD
+	FSMC_A18(PD13) -> LCD_RS
+	FSMC_D0(PD14) -> LCD_D0
+	FSMC_D1(PD15) -> LCD_D1
+	FSMC_D2(PD0) -> LCD_D2
+	FSMC_D3(PD1) -> LCD_D3
+	FSMC_D4(PE7) -> LCD_D4
+	FSMC_D5(PE8) -> LCD_D5
+	FSMC_D6(PE9) -> LCD_D6
+	FSMC_D7(PE10) -> LCD_D7
+	PD12 -> LCD_RST

<h1 align="center"> LVGL - Light and Versatile Graphics Library</h1>

<p align="center">
<img src="https://lvgl.io/assets/images/lvgl_widgets_demo.gif">
</p>

<p align="center">
LVGL provides everything you need to create an embedded GUI with easy-to-use graphical elements, beautiful visual effects and a low memory footprint.
</p>

<h4 align="center">
<a href="https://lvgl.io">Website </a> &middot;
<a href="https://docs.lvgl.io/">Docs</a> &middot;
<a href="https://forum.lvgl.io">Forum</a>  &middot;
<a href="https://lvgl.io/services">Services</a>  &middot;
<a href="https://docs.lvgl.io/master/examples.html">Interactive examples</a>
</h4>



---

#### Table of content
- [Overview](#overview)
- [Get started](#get-started)
- [Examples](#examples)
- [Services](#services)
- [Contributing](#contributing)

## Overview
### Features
* Powerful [building blocks](https://docs.lvgl.io/master/widgets/index.html): buttons, charts, lists, sliders, images, etc.
* Advanced graphics engine: animations, anti-aliasing, opacity, smooth scrolling, blending modes, etc
* Supports [various input devices](https://docs.lvgl.io/master/overview/indev.html): touchscreen, mouse, keyboard, encoder, buttons, etc.
* Supports [multiple displays](https://docs.lvgl.io/master/overview/display.html)
* Hardware independent, can be use with any microcontroller and display
* Scalable to operate with little memory (64 kB Flash, 16 kB RAM)
* Multi-language support with UTF-8 handling, CJK, Bidirectional and Arabic script support
* Fully customizable graphical elements via [CSS-like styles](https://docs.lvgl.io/master/overview/style.html)
* Powerful layouts inspired by CSS: [Flexbox](https://docs.lvgl.io/master/layouts/flex.html) and [Grid](https://docs.lvgl.io/master/layouts/grid.html)
* OS, External memory and GPU are supported but not required. (built in support for STM32 DMA2D, SWM341 DMA2D, and NXP PXP and VGLite)
* Smooth rendering even with a [single frame buffer](https://docs.lvgl.io/master/porting/display.html)
* Written in C and compatible with C++
* Micropython Binding exposes [LVGL API in Micropython](https://blog.lvgl.io/2019-02-20/micropython-bindings)
* [Simulator](https://docs.lvgl.io/master/get-started/platforms/pc-simulator.html) to develop on PC without embedded hardware
* 100+ simple [Examples](https://github.com/lvgl/lvgl/tree/master/examples)
* [Documentation](http://docs.lvgl.io/) and API references online and in PDF

### Requirements
Basically, every modern controller (which is able to drive a display) is suitable to run LVGL. The minimal requirements are:
<table>
  <tr>
    <td> <strong>Name</strong> </td>
    <td><strong>Minimal</strong></td>
    <td><strong>Recommended</strong></td>
  </tr>
  <tr>
    <td><strong>Architecture</strong></td>
    <td colspan="2">16, 32 or 64 bit microcontroller or processor</td>
  </tr>
  <tr>
    <td> <strong>Clock</strong></td>
    <td> &gt; 16 MHz </td>
    <td> &gt; 48 MHz</td>
  </tr>

  <tr>
    <td> <strong>Flash/ROM</strong></td>
    <td> &gt; 64 kB </td>
    <td> &gt; 180 kB</td>
  </tr>

  <tr>
    <td> <strong>Static RAM</strong></td>
    <td> &gt; 16 kB </td>
    <td> &gt; 48 kB</td>
  </tr>

  <tr>
    <td> <strong>Draw buffer</strong></td>
    <td> &gt; 1 &times; <em>hor. res.</em> pixels </td>
    <td> &gt; 1/10 screen size </td>
  </tr>

  <tr>
    <td> <strong>Compiler</strong></td>
    <td colspan="2"> C99 or newer </td>
  </tr>
</table>

*Note that the memory usage might vary depending on the architecture, compiler and build options.*

## Get started
This list shows the recommended way of learning the library:
1. Check the [Online demos](https://lvgl.io/demos) to see LVGL in action (3 minutes)
2. Read the [Introduction](https://docs.lvgl.io/master/intro/index.html) page of the documentation (5 minutes)
3. Get familiar with the basics on the [Quick overview](https://docs.lvgl.io/master/get-started/quick-overview.html) page (15 minutes)
4. Set up a [Simulator](https://docs.lvgl.io/master/get-started/platforms/pc-simulator.html) (10 minutes)
5. Try out some [Examples](https://github.com/lvgl/lvgl/tree/master/examples)
6. Port LVGL to a board. See the [Porting](https://docs.lvgl.io/master/porting/index.html) guide or check the ready to use [Projects](https://github.com/lvgl?q=lv_port_)
7. Read the [Overview](https://docs.lvgl.io/master/overview/index.html) page to get a better understanding of the library (2-3 hours)
8. Check the documentation of the [Widgets](https://docs.lvgl.io/master/widgets/index.html) to see their features and usage
9. If you have questions go to the [Forum](http://forum.lvgl.io/)
10. Read the [Contributing](https://docs.lvgl.io/master/CONTRIBUTING.html) guide to see how you can help to improve LVGL (15 minutes)

## Examples

For more examples see the [examples](https://github.com/lvgl/lvgl/tree/master/examples) folder.

![LVGL button with label example](https://github.com/lvgl/lvgl/raw/master/docs/misc/btn_example.png)

### C
```c
lv_obj_t * btn = lv_btn_create(lv_scr_act());                   /*Add a button to the current screen*/
lv_obj_set_pos(btn, 10, 10);                                    /*Set its position*/
lv_obj_set_size(btn, 100, 50);                                  /*Set its size*/
lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL); /*Assign a callback to the button*/

lv_obj_t * label = lv_label_create(btn);                        /*Add a label to the button*/
lv_label_set_text(label, "Button");                             /*Set the labels text*/
lv_obj_center(label);                                           /*Align the label to the center*/
...

void btn_event_cb(lv_event_t * e)
{
  printf("Clicked\n");
}
```

## Services
LVGL Kft was established to provide a solid background for LVGL library. We offer several type of services to help you in UI development:
- Graphics design
- UI implementation
- Consulting/Support

For more information see https://lvgl.io/services
Feel free to contact us if you have any questions.


## Contributing
LVGL is an open project and contribution is very welcome. There are many ways to contribute from simply speaking about your project, through writing examples, improving the documentation, fixing bugs to hosting your own project under the LVGL organization.

For a detailed description of contribution opportunities visit the [Contributing](https://docs.lvgl.io/master/CONTRIBUTING.html) section of the documentation.

