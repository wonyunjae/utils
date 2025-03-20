import cv2
import time
import os

def capture_stereo_images(left_cam_id=0, right_cam_id=2, output_dir='./stereo_captured'):
    # 출력 디렉토리 생성
    os.makedirs(output_dir, exist_ok=True)
    
    # 두 카메라 열기
    left_cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
    right_cap = cv2.VideoCapture("/dev/video2", cv2.CAP_V4L2)
    
    # 카메라 설정 - 해상도 조정 (메모리 사용량 감소)
    width, height = 640, 480  # 낮은 해상도 사용
    left_cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    left_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    right_cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    right_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    if not left_cap.isOpened() or not right_cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return
    
    print("두 카메라 영상을 확인합니다. 캡처하려면 's'를 누르고, 종료하려면 'q'를 누르세요.")
    
    while True:
        # 프레임 읽기
        ret_left, left_frame = left_cap.read()
        ret_right, right_frame = right_cap.read()
        
        if not ret_left or not ret_right:
            print("프레임을 읽을 수 없습니다.")
            break
        
        # 좌우 이미지 나란히 표시
        combined = cv2.hconcat([left_frame, right_frame])
        cv2.putText(combined, "Left Camera", (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(combined, "Right Camera", (width + 50, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Stereo Cameras', combined)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # 이미지 저장
            timestamp = int(time.time())
            left_path = os.path.join(output_dir, f'left_{timestamp}.png')
            right_path = os.path.join(output_dir, f'right_{timestamp}.png')
            
            cv2.imwrite(left_path, left_frame)
            cv2.imwrite(right_path, right_frame)
            print(f"이미지 저장 완료: {left_path}, {right_path}")
            
            # 카메라 내부 파라미터 (예시 값 - 실제로는 캘리브레이션 필요)
            # 실제 C270 카메라에 더 적합한 대략적인 값으로 수정
            K = [1000, 0, width/2, 0, 1000, height/2, 0, 0, 1]  # 근사값
            baseline = 0.1  # 카메라 간 거리 (미터 단위) - 측정 필요
            
            # K.txt 파일 저장
            with open(os.path.join(output_dir, 'K.txt'), 'w') as f:
                f.write(' '.join(map(str, K)) + '\n')
                f.write(str(baseline))
            
            print(f"내부 파라미터 저장 완료: {os.path.join(output_dir, 'K.txt')}")
            
    # 자원 해제
    left_cap.release()
    right_cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_stereo_images(left_cam_id=0, right_cam_id=1)