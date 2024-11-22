# 3.color_detect - By: Janghun Hyeon - Thu Nov 21 2024

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

threshold_red = (222,230,154,154,33,41)   # ex) 빨간색 임계값 설정 : (100, 180, 0, 50, 0, 50)


def find_blobs(img, threshold):
    width = img.width()
    height = img.height()
    blobs = []

    for y in range(0,height,5): # 간격 (5 pixel)을 두고 스캔
        for x in range(0,width,5):
            pixel = img.get_pixel(x, y)
            r, g, b = pixel

            if pixel == threshold:
                blobs.append(Blob(x,y,5,5)) # Blob 추가
    return blobs


while(True):
    clock.tick()
    img = sensor.snapshot()
    blobs = find_blobs(img, threshold_red)

    for blob in blobs:
        # blob을 화면에 표시
        img.draw_rectangle(blob.rect(), color=(255,0,0,)) #blob 경계에 빨간 사각형 그리기
        img.draw_cross(blob.x + blob.w //2, blob.y + blob.h//2, color=(0,255,0)) #blob 중심에 초록색 십자가

        # blob 정보 출력
        print("Blob [면적: %d, 중심 좌표: (%d, %d), 가로: %d, 세로: %d]" %(blob.area(), blob.x + blob.w//2, blob.y + blob.h//2, blob.w, blob.h))

    # print(clock.fps())
