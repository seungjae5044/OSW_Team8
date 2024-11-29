# 2.make_blob - By: Janghun Hyeon - Thu Nov 21 2024

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()



class Blob:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def rect(self):
        return (self.x, self.y, self.w, self.h)

    def area(self):
        return self.w * self.h


def create_random_blob():
    return Blob(90, 90, 50, 50) #x=30, y=30에서 시작하는 가로 50, 세로 50 크기의 Blob을 만든다.

while(True):
    clock.tick()
    img = sensor.snapshot()

    blob = create_random_blob()

    img.draw_rectangle(blob.rect(), color=(255,0,0,)) #blob 경계에 빨간 사각형 그리기
    img.draw_cross(blob.x + blob.w //2, blob.y + blob.h//2, color=(0,255,0)) #blob 중심에 초록색 십자가

    # blob 정보 출력\
    print("Blob [면적: %d, 중심 좌표: (%d, %d), 가로: %d, 세로: %d]" %(blob.area(), blob.x + blob.w//2, blob.y + blob.h//2, blob.w, blob.h))

    print(clock.fps())
