# 소갈주머니
------------------------------------------------------
## 2020 미니드론 자율비행 경진대회
------------------------------------------------------
### [✔ 대회 진행 전략] 

* 여러가지 상황에 대한 변수를 가장 최우선적으로 고려함
* 1차 2차 3차 시도에 대한 각각 코드 준비
* HSV값으로 색 인식
* 홀을 완벽하게 인식하는 것으로 기본으로 하여 링 통과
* 침식과 팽창을 사용한 노이즈 제거
* 링 중앙점 인식 정확도를 높임
------------------------------------------------------
### [✔ 알고리즘 설명]
![미니드론알고리즘](https://user-images.githubusercontent.com/61452782/87249449-1b490780-c49a-11ea-8aa9-996f42cff3ce.jpg)
------------------------------------------------------
1. 이륙             
2. 링의 중앙점에 드론이 **위치 해 있는지 인식**
* 링의 중앙점에 드론이 위치 해 있을 경우
  + **다음 단계**
* 링의 중앙점에 드론이 위치하지 않을 경우
  + 상하좌우 조정        
3. 드론이 링의 중앙점에 위치하고 직진비행 할 수 있는 최적 거리인지 판단(링을 통과하여 무리 없이 비행할 수 있는지)
* 드론이 링의 중앙점에 위치하지 않고 최적 거리 밖에 있다면 **전진** 하여 드론이 링의 중앙점에 위치하고 최적 거리로 오도록 유도 -> 이륙 이후 알고리즘으로 다시 되돌아감
4. 최적 거리내에 드론이 있다면 **링 통과**
5. 링 통과 후 원의 색상 인식
* **파란색일 경우**
  + 착륙   
* **빨간색일 경우**
  + 좌로 90도 회전   
6. 좌회전 후 설정한 값만큼 **전진**   
7. 드론이 착륙하지 않았을 경우 알고리즘 초기단계 

------------------------------------------------------
### [✔ 소스코드 설명] 
----------------------------------------------------
#### ✔ 1차 시도(기본 코드로 사용할 예정)
---------------------------------------------------
* 변수 선언 및 임계값 설정
```
droneObj = ryze()

global min_h;   global max_h;
min_h = 0.225;  max_h = 0.405;
global dist_to_cir;
dist_to_cir = 2.45;
global move_dist;
move_dist = 0;

```

* 이륙
```
takeoff(droneObj);
Move(droneObj, 0.3, "up");
```

* 텔로 카메라 프레임 수신
```
cameraObj = camera(droneObj);
preview(cameraObj);
[frame,ts] = snapshot(cameraObj);
```

* 링 홀의 중앙 좌표값 검출
```
[hall_frame, x, y] = loc_recog(frame);
while 1
    [frame,ts] = snapshot(cameraObj);
    [hall_frame, x, y] = loc_recog(frame)
    if isnan(x) || isnan(y) || x-5 < 0 || y-5 < 0
        continue;
    end
```
---------------------------------------------------------
* 링의 중앙점이 최적 거리 내에 있을 때 직진 2.3M
```
    if ((x >= 450) && (x <= 550)) && ((y >= 110) && (y <= 190))
        dist = 2.3;
        dir = "forward";
        disp("forward");
        Move(droneObj, dist, dir);
```

* 1단계 2단계 각각 링 통과 후 다음 링 홀 위치 파악 후 이동
```
       if level < 3
            Rotate(droneObj, -90);
            [frame,ts] = snapshot(cameraObj);
            [hall_frame, x, y] = loc_recog(frame);
            
            if x < 500
                Move(droneObj, 1, dir);
                dir = "left";
                Move(droneObj, 0.3, dir);
            else
                Move(droneObj, 1, dir);
                dir = "right";
                Move(droneObj, 0.3, dir);
            end
            level = level + 1;
```
--------------------------------------------------------------
* 링이 너무 높아 화면에 안나올 경우
```
                if isnan(x) || isnan(y) || x-5 < 0 || y-5 < 0
                    Move(droneObj, 0.4, "up");
                    
                    [frame,ts] = snapshot(cameraObj);
                    [hall_frame, x, y] = loc_recog(frame);                                    
                    
                    if isnan(x) || isnan(y) || x-5 < 0 || y-5 < 0
                        Move(droneObj, 0.7, "down");
                        break;
                    end
                end
```
* 링 통과 후 90도 회전하고 드론 위치 설정
                if x < 500 && y < 150
                    Move(droneObj, 1.1, "forward");
                    Move(droneObj, 0.3, "left");
                    Move(droneObj, 0.3, "up");
                    break;
                elseif x < 500 && y > 150
                    Move(droneObj, 1.1, "forward");
                    Move(droneObj, 0.3, "left");
                    Move(droneObj, 0.3, "down");
                    break;
                elseif x > 500 && y < 150
                    Move(droneObj, 1.1, "forward");
                    Move(droneObj, 0.3, "right");
                    Move(droneObj, 0.3, "up");
                    break;
                elseif x > 500 && y > 150
                    Move(droneObj, 1.1, "forward");
                    Move(droneObj, 0.3, "right");
                    Move(droneObj, 0.3, "down");
                    break;
                end
```
* 원이 드론시야에 들어오지 않을 때
```
            elseif cir_num == 3
                while force_cir_noncheck == 0
```
* 좌우 회전 하여 시야 확보 후에 원이 시야에 들어오지 않을 때
```
Rotate(droneObj, 20);
                   cir_num = Cir_Check(cameraObj);
                   if cir_num ~= 3
                       force_cir_noncheck = 1;
                   end
```
* 원 위치
```
                   Rotate(droneObj, -20);
```

* 3단계 링 통과 후 착지
```
        elseif level == 3
            land(droneObj);
            break;
        end
```

* 링의 중앙점이 최적 거리 내에 없을 때 중앙점의 좌표에 따라 드론을 이동
```
    else
        dist = 0.2;
        x_diff = x - 500;
        y_diff = y - 150;

        if y_diff > 30
            dir = "down";
            disp("down");
            Move(droneObj, dist, dir);
        elseif y_diff < -30
            dir = "up";
            disp("up");
            Move(droneObj, dist, dir);
        end
        
        if x_diff > 30
            dir = "right";
            disp("right");
            Move(droneObj, dist, dir);
        elseif x_diff < -30
            dir = "left";
            disp("left");
            Move(droneObj, dist, dir);
        end
    end
end
```

* 위치 인식
```
function [hall_frame, x, y] = loc_recog(frame)
    global min_h;
    global max_h;
    
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    detect_green = (min_h < h) & (h < max_h);
    
```
--------------------------------------------------------------------------
* 픽셀수가 일정 개수보다 적은 연결성분 제거
```
    bw = bwareaopen(detect_green, 1000);
```

* 침식
```
    se = strel('line', 20, 0);
    bw = imerode(bw,se);
```

* 팽창
```
    bw = imdilate(bw, se);
```

* 링의 중앙점 검출
```
    [width, height] = size(bw);
    
    bw(:, 1) = 1;
    bw(:, width) = 1;
    bw(1, :) = 1;
    
    bw2 = imfill(bw, 'holes');
    
    bw3 = bw2 - bw;
    hall_frame = bw3;

    [row, col] = find(bw3);
    row = sort(row);
    col = sort(col);
    y = int16(median(row));
    x = int16(median(col));
end
```

* 입력되는 방향에 따라 드론을 이동
```
function rtn = Move(droneObj, dist, dir)
   global move_dist;
    if dir == "forward"
        moveforward(droneObj, 'Distance', dist);
        move_dist = move_dist + dist;
    elseif dir == "back"
        moveback(droneObj, 'Distance', dist);
    elseif dir == "right"
        moveright(droneObj, 'Distance', dist);
    elseif dir == "left"
        moveleft(droneObj, 'Distance', dist);
    elseif dir == "up"
        moveup(droneObj, 'Distance', dist);
    elseif dir == "down"
        movedown(droneObj, 'Distance', dist);
    end
    pause(0.5);
    rtn = "";
end
```

* 입력 받은 각도만큼 드론을 회전
```
function rtn = Rotate(droneObj,ang)
    turn(droneObj, deg2rad(ang));
    rtn = "";
end
```
------------------------------------------------
#### ✔ 2차 시도
------------------------------------------------
```
코드
```
------------------------------------------------
#### ✔ 3차 시도
-----------------------------------------------
```
코드
```
