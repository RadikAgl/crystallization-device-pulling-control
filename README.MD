# Управление штоком кристаллизационной установки

Скрипт используется для управления драйвером штока на основе arduino nano ATmega168 кристаллизационной установки. 

## Особенности

Функциональность:
* возможность запуска протяжки штока как через [Термодат](https://www.termodat.ru/), так и в ручном режиме;
* используются энкодер KY-040 и и дисплей LCD1602(HD44780);
* возможность задавать многошаговый режим протяжки(до 10), в каждом из которых своя скорость и длина протяжки;
* скорость вытягивания задается в мм/ч с шагом 0,1 мм/ч;
* длина протяжки задается в см с шагом 0,1 см;
* возможность сохранять значения скоростей, длин и калибровочного коэффициента в  EEPROM;  
* при включении все значения загружаются из EEPROM.

## Requirements

* <LiquidCrystal.h>
* <TimerOne.h>
* <EEPROM.h>
