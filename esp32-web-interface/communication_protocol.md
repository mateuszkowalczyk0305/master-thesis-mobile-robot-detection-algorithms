# Communication Protocol:

## ESP32 as master:

`Methodes:` 

- IR: #M:I;
- Ultrasonic: #M:U;
- LiDAR: #M:L;
- Fusion: #M:F;

`Motion:`

- Front: #D:1,x;
- Back:  #D:2,x;
- Right: #D:3,x;
- Left:  #D:4,x; 
- Stop:  #D,5,x;

x - number 0-9. 

0 = min speed, 9 = max speed.

## STM32 as slave (responses):

`Argument:`

- time_ms = uint number;

`Methodes:`
- IR: #M:I,time_ms;
- Ultrasonic: #M:U,time_ms;
- LiDAR: #M:L,time_ms;
- Fusion: #M:F,time_ms;

`Motion:`
- #D:OK;

## UI:
![UI](ui.png)

## UI in service mode:
![UI](ui2.png)