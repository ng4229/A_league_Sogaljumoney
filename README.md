# 소갈주머니
------------------------------------------------------
## 2020 미니드론 자율비행 경진대회
### [✔ 대회 진행 전략] 
* 여러가지 상황에 대한 변수를 가장 최우선적으로 고려함.
* HSV값으로 색 인식
* 홀을 완벽하게 인식하는 것으로 기본으로 하여 링 통과
* 침식과 팽창을 사용한 노이즈 제거
* 링 중앙점 인식 정확도를 높임
### [✔ 알고리즘 설명] 
![미니드론알고리즘](https://user-images.githubusercontent.com/61452782/87249449-1b490780-c49a-11ea-8aa9-996f42cff3ce.jpg)

1. 이륙
2. 링의 중앙점에 드론이 **위치 해 있는지 인식**
* 링의 중앙점에 드론이 위치 해 있을 경우
  + **다음 단계**
* 링의 중앙점에 드론이 위치하지 않을 경우
  + 상하좌우 조정
3. 드론이 링의 중앙점에 위치하고 직진비행 할 수 있는 최적 거리인지 판단(링을 통과하여 무리 없이 비행할 수 있는지)
* 드론이 링의 중앙점에 위치하지 않고 최적 거리 밖에 있다면 전진하여 드론이 링의 중앙점에 위치하고 최적 거리로 오도록 유도
4.
5.
6.
7.


### [✔ 소스코드 설명] 
* 변수 선언
```
droneObj = ryze()

global min_h;
global max_h;
level = 1;
min_h = 0.225;
max_h = 0.405;
```

* 이륙
```
takeoff(droneObj);
```

* 상승
```
moveup(droneObj, 'Distance', 0.3);
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
    pause(2);
    [frame,ts] = snapshot(cameraObj);
    [hall_frame, x, y] = loc_recog(frame);
    
    left = 0;
    right = 0;
    if nnz(hall_frame) < 50
        Rotate(droneObj, -30);
        left = 1;
        [hall_frame, x, y] = loc_recog(frame);
        continue;
    end
    if left == 1
        Rotate(droneObj, )
        
    
    if isnan(x) || isnan(y) || x-5 < 0 || y-5 < 0
        continue;
    end
```
    
* 삭제 예정 항목(중앙 점 찍기)
```
    for r = x-5:x+5
       for c = y-5:y+5
           hall_frame(c, r) = 0;
       end
    end
    imshow(hall_frame);
```
    
* 직진 2.5M
```
    if ((x >= 420) && (x <= 580)) && ((y >= 100) && (y <= 200))
        dist = 2.5;
        dir = "forward";
        disp("forward");
        Move(droneObj, dist, dir);
```
     
```
        if level < 3
            Rotate(droneObj, -90);
            Move(droneObj, 1, dir);
            level = level + 1;
        elseif level == 3
            land(droneObj);
            break;
        end
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

function [hall_frame, x, y] = loc_recog(frame)
    global min_h;
    global max_h;
    
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    
    detect_green = (min_h < h) & (h < max_h);
    
    % 픽셀수가 일정 개수보다 적은 연결성분 제거
    bw = bwareaopen(detect_green, 1000);
    
    %침식
    se = strel('line', 20, 0);
    bw = imerode(bw,se);
    
    % 팽창
    bw = imdilate(bw, se);
    [width, height] = size(bw);
    
    for i = 1:height
        bw(i, 1) = 1;
        bw(i, width) = 1;
    end
    
    for i = 1:width
        bw(1, i) = 1;
    end
    
    bw2 = imfill(bw, 'holes');
    
    bw3 = bw2 - bw;
    hall_frame = bw3;

    [row, col] = find(bw3);
    row = sort(row);
    col = sort(col);
    y = int16(median(row));
    x = int16(median(col));
end

function rtn = Move(droneObj, dist, dir)
    % 입력되는 방향에 따라 드론을 이동
    if dir == "forward"
        moveforward(droneObj, 'Distance', dist);
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
    rtn = "";
end

function rtn = Rotate(droneObj,ang)
    % 입력 받은 각도만큼 드론을 회전
    turn(droneObj, deg2rad(ang));
    rtn = "";
end
```
