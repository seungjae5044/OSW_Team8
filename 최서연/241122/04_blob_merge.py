# 4.blob_merge - By: Janghun Hyeon - Thu Nov 21 2024

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

threshold = (200, 250, 150, 180, 20, 50)   # 빨간색 임계값 설정


def find_blobs(img, threshold):

    width = img.width()
    height = img.height()
    blobs = []

    for y in range(0,height,5): # 간격 (5 pixel)을 두고 스캔
        for x in range(0,width,5):
            pixel = img.get_pixel(x, y)
            r, g, b = pixel

            if threshold[0] <= r <= threshold[1] and threshold[2] <= g <= threshold[3] and threshold[4] <= b <= threshold[5]:
                blobs.append(Blob(x,y,5,5)) # Blob 추가
    return blobs



def merge_blobs(blobs, distance_threshold=10):
    merged_blobs = []

    while blobs:
        current_blob = blobs.pop(0)
        merged = False

        for merged_blob in merged_blobs:
            dx = abs(current_blob.x - merged_blob.x)
            dy = abs(current_blob.y - merged_blob.y)

            if dx <= distance_threshold and dy <= distance_threshold:
                x1 = min(current_blob.x, merged_blob.x)
                y1 = min(current_blob.y, merged_blob.y)
                x2 = max(current_blob.x + current_blob.w, merged_blob.x + merged_blob.w)
                y2 = max(current_blob.y + current_blob.h, merged_blob.y + merged_blob.h)

                merged_blob.x = x1
                merged_blob.y = y1
                merged_blob.w = x2-x1
                merged_blob.h = y2-y1
                merged = True
                break

        if not merged:
            merged_blobs.append(current_blob)

    return merged_blobs



while(True):
    clock.tick()
    img = sensor.snapshot()

    blobs = find_blobs(img, threshold)

    # 병합된 blob 처리
    merged_blobs = merge_blobs(blobs)

    for blob in merged_blobs:
        # blob을 화면에 표시
        img.draw_rectangle(blob.rect(), color=(0,255,0)) #blob 경계에 녹색 사각형 그리기
        img.draw_cross(blob.x + blob.w //2, blob.y + blob.h//2, color=(255,0,0)) #blob 중심에 초록색 십자가

        # blob 정보 출력
        print("Blob [면적: %d, 중심 좌표: (%d, %d), 가로: %d, 세로: %d]" %(blob.area(), blob.x + blob.w//2, blob.y + blob.h//2, blob.w, blob.h))

    #print(clock.fps())
