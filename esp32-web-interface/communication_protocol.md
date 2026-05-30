# Communication Protocol:

## ESP32 as master:

`Methodes:` 

- IR: #M:I;
- Ultrasonic: #M:U;
- LiDAR: #M:L;
- Fusion: #M:F;

`Motion:`

- Front: #D:1;
- Back:  #D:2;
- Right: #D:3;
- Left:  #D:4; 

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