import tkinter as tk
import subprocess
import time
import threading
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from collections import deque

class LocalizationUI:
    
    def __init__(self):
        # 기본 설정
        self.foundation_stereo_path = '/home/smarthc/FoundationStereo'
        self.glim_config_dir = '/home/smarthc/ros2_ws/src/glim/config'
        
        # 프로세스 추적
        self.processes = {}
        
        # 데이터 저장 (실시간 플롯용)
        self.utm_data = {'x': deque(maxlen=100), 'y': deque(maxlen=100), 'time': deque(maxlen=100)}
        self.heading_data = deque(maxlen=100)
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.current_heading = 0.0
        
        print("LocalizationUI initialized")
    
    def setup_ui(self):
        """UI 설정"""
        print("Creating UI window...")
        self.root = tk.Tk()
        self.root.title("🚗 GLIM Localization System Controller")
        self.root.geometry("1400x900+50+50")
        self.root.configure(bg='#f0f0f0')
        
        # 종료 처리
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # UI 구성 요소 생성
        self.create_control_panel()
        self.create_visualization_panel()
        self.create_log_panel()
        
        print("UI window created successfully!")
        return self.root
    
    def create_control_panel(self):
        """제어 패널 - 상단 고정"""
        control_frame = tk.LabelFrame(self.root, text="Node Control", font=('Arial', 12, 'bold'))
        control_frame.pack(fill='x', padx=10, pady=5)
        
        # 노드별 제어 버튼들
        nodes = [
            ('GLIM', 'glim'),
            ('Google Earth', 'google_earth'),
            ('FoundationStereo', 'foundation_stereo'),
            ('TAE Localization', 'tae_localization')
        ]
        
        for i, (name, node_id) in enumerate(nodes):
            frame = tk.Frame(control_frame)
            frame.pack(fill='x', padx=5, pady=3)
            
            # 노드 이름
            tk.Label(frame, text=name, font=('Arial', 10, 'bold'), width=15, anchor='w').pack(side='left')
            
            # 시작 버튼
            start_btn = tk.Button(frame, text="Start", bg='green', fg='white', width=7,
                                command=lambda n=node_id: self.start_node(n))
            start_btn.pack(side='left', padx=3)
            
            # 정지 버튼
            stop_btn = tk.Button(frame, text="Stop", bg='red', fg='white', width=7,
                               command=lambda n=node_id: self.stop_node(n))
            stop_btn.pack(side='left', padx=3)
            
            # 상태 표시
            status_label = tk.Label(frame, text="● Stopped", fg='red', width=10)
            status_label.pack(side='left', padx=5)
            
            # 상태 레이블 저장
            if not hasattr(self, 'status_labels'):
                self.status_labels = {}
            self.status_labels[node_id] = status_label
        
        # 전체 제어 버튼
        all_control_frame = tk.Frame(control_frame)
        all_control_frame.pack(fill='x', pady=5)
        
        tk.Button(all_control_frame, text="Start All", bg='blue', fg='white', width=10,
                 command=self.start_all_nodes).pack(side='left', padx=5)
        tk.Button(all_control_frame, text="Stop All", bg='orange', fg='white', width=10,
                 command=self.stop_all_nodes).pack(side='left', padx=5)
        
        # 실행 중인 프로세스 수
        self.process_count_label = tk.Label(all_control_frame, text="Running processes: 0", 
                                          font=('Arial', 10))
        self.process_count_label.pack(side='right', padx=10)
    
    def create_visualization_panel(self):
        """시각화 패널 - 메인 영역"""
        viz_frame = tk.LabelFrame(self.root, text="Real-time Visualization", font=('Arial', 12, 'bold'))
        viz_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # 4분할 영역 생성
        main_container = tk.Frame(viz_frame)
        main_container.pack(fill='both', expand=True, padx=5, pady=5)
        
        # 상단 영역 (2분할)
        top_frame = tk.Frame(main_container)
        top_frame.pack(fill='both', expand=True)
        
        # 왼쪽 상단: UTM 경로 플롯
        self.create_utm_plot(top_frame)
        
        # 오른쪽 상단: 시스템 정보
        self.create_system_info(top_frame)
        
        # 하단 영역 (2분할)
        bottom_frame = tk.Frame(main_container)
        bottom_frame.pack(fill='both', expand=True)
        
        # 왼쪽 하단: FoundationStereo 정보
        self.create_foundation_info(bottom_frame)
        
        # 오른쪽 하단: GLIM 정보
        self.create_glim_info(bottom_frame)
    
    def create_utm_plot(self, parent):
        """UTM 경로 실시간 플롯"""
        utm_frame = tk.LabelFrame(parent, text="🗺️ Vehicle Path (UTM)", font=('Arial', 10, 'bold'))
        utm_frame.pack(side='left', fill='both', expand=True, padx=2, pady=2)
        
        # matplotlib 플롯 생성
        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.ax.set_title('Vehicle Trajectory')
        self.ax.set_xlabel('UTM X (m)')
        self.ax.set_ylabel('UTM Y (m)')
        self.ax.grid(True, alpha=0.3)
        
        # 경로 선과 현재 위치 점
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Path')
        self.current_point, = self.ax.plot([], [], 'ro', markersize=8, label='Current')
        self.ax.legend()
        
        # tkinter에 matplotlib 임베드
        self.canvas = FigureCanvasTkAgg(self.fig, utm_frame)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        
        # 위치 정보 텍스트
        self.position_info = tk.Label(utm_frame, text="Position: Not Available", 
                                     font=('Arial', 9), bg='white')
        self.position_info.pack(fill='x', padx=5, pady=2)
    
    def create_system_info(self, parent):
        """시스템 정보 패널"""
        info_frame = tk.LabelFrame(parent, text="📊 System Information", font=('Arial', 10, 'bold'))
        info_frame.pack(side='right', fill='both', expand=True, padx=2, pady=2)
        
        # 텍스트 위젯으로 시스템 정보 표시
        self.system_text = tk.Text(info_frame, height=15, font=('Consolas', 9))
        self.system_text.pack(fill='both', expand=True, padx=5, pady=5)
        
        # 초기 정보 표시
        self.update_system_info()
    
    def create_foundation_info(self, parent):
        """FoundationStereo 정보 패널"""
        foundation_frame = tk.LabelFrame(parent, text="🎯 FoundationStereo Status", font=('Arial', 10, 'bold'))
        foundation_frame.pack(side='left', fill='both', expand=True, padx=2, pady=2)
        
        # FoundationStereo 상태 정보
        info_text = """
Camera Configuration:
• Left Camera ID: 4
• Right Camera ID: 6
• Point Cloud Interval: 30 frames
• Output Directory: /tmp/foundation_stereo_output/

Status Information:
• Camera Status: Not Connected
• Frame Processing: 0 FPS
• Point Cloud Generation: Inactive
• Output Files: 0 PLY files generated

Recent Activity:
• Waiting for camera initialization...
        """
        
        self.foundation_text = tk.Text(foundation_frame, height=15, font=('Consolas', 9))
        self.foundation_text.pack(fill='both', expand=True, padx=5, pady=5)
        self.foundation_text.insert('1.0', info_text)
    
    def create_glim_info(self, parent):
        """GLIM 정보 패널"""
        glim_frame = tk.LabelFrame(parent, text="🧭 GLIM Localization Status", font=('Arial', 10, 'bold'))
        glim_frame.pack(side='right', fill='both', expand=True, padx=2, pady=2)
        
        # GLIM 상태 정보
        info_text = """
SLAM Status:
• Tracking State: NOT_INITIALIZED
• Map Points: 0
• KeyFrames: 0
• Loop Closures: 0

Sensor Information:
• IMU Status: Waiting for data
• Camera Status: Waiting for images
• Feature Tracking: Inactive

Performance Metrics:
• Processing Rate: 0 Hz
• Memory Usage: 0 MB
• Trajectory Points: 0

Recent Activity:
• System waiting for initialization...
        """
        
        self.glim_text = tk.Text(glim_frame, height=15, font=('Consolas', 9))
        self.glim_text.pack(fill='both', expand=True, padx=5, pady=5)
        self.glim_text.insert('1.0', info_text)
    
    def create_log_panel(self):
        """로그 패널 - 하단 고정"""
        log_frame = tk.LabelFrame(self.root, text="System Logs", font=('Arial', 12, 'bold'))
        log_frame.pack(fill='x', padx=10, pady=5)
        
        # 스크롤 가능한 텍스트
        import tkinter.scrolledtext as scrolledtext
        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, font=('Consolas', 9))
        self.log_text.pack(fill='x', padx=5, pady=5)
        
        # 초기 메시지
        self.log_message("System ready. Start nodes to begin localization.")
    
    def update_utm_plot(self):
        """UTM 플롯 업데이트"""
        if len(self.utm_data['x']) > 0:
            # 경로 업데이트
            self.path_line.set_data(self.utm_data['x'], self.utm_data['y'])
            
            # 현재 위치 업데이트
            current_x = self.utm_data['x'][-1]
            current_y = self.utm_data['y'][-1]
            self.current_point.set_data([current_x], [current_y])
            
            # 축 범위 자동 조정
            if len(self.utm_data['x']) > 1:
                margin = 10  # 10m 여유
                x_min, x_max = min(self.utm_data['x']) - margin, max(self.utm_data['x']) + margin
                y_min, y_max = min(self.utm_data['y']) - margin, max(self.utm_data['y']) + margin
                self.ax.set_xlim(x_min, x_max)
                self.ax.set_ylim(y_min, y_max)
            
            # 위치 정보 업데이트
            self.position_info.config(text=f"Position: X={current_x:.1f}m, Y={current_y:.1f}m")
            
            # 캔버스 업데이트
            self.canvas.draw()
    
    def update_system_info(self):
        """시스템 정보 업데이트"""
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        
        info_text = f"""=== System Status ===
Current Time: {current_time}

Running Processes: {len(self.processes)}
{self.get_process_list()}

Position Information:
• UTM X: {self.current_position['x']:.3f} m
• UTM Y: {self.current_position['y']:.3f} m
• Heading: {self.current_heading:.3f} rad ({np.degrees(self.current_heading):.1f}°)

Data Collection:
• Path Points: {len(self.utm_data['x'])}
• Update Rate: Real-time

Memory Usage:
• Total Points: {len(self.utm_data['x'])}
• Buffer Size: 100 points (auto-cleanup)

Network Status:
• ROS2 Topics: Active
• Node Communication: Healthy
"""
        
        self.system_text.delete('1.0', tk.END)
        self.system_text.insert('1.0', info_text)
    
    def get_process_list(self):
        """실행 중인 프로세스 목록 반환"""
        if not self.processes:
            return "• No processes running"
        
        process_list = ""
        for name, process in self.processes.items():
            if process.poll() is None:  # 실행 중
                process_list += f"• {name}: Running (PID: {process.pid})\n"
            else:  # 종료됨
                process_list += f"• {name}: Terminated\n"
        
        return process_list
    
    def start_node(self, node_name):
        """노드 시작 - 실제 명령어 실행"""
        try:
            if node_name == 'glim':
                cmd = ['ros2', 'run', 'glim_ros', 'glim_rosnode']
            elif node_name == 'google_earth':
                cmd = ['ros2', 'run', 'local_pkg', 'google_earth_visualizer']
            elif node_name == 'foundation_stereo':
                # Conda 환경에서 실행
                script = f"""#!/bin/bash
source /home/smarthc/anaconda3/bin/activate foundation_stereo
cd {self.foundation_stereo_path}
python3 scripts/trt_videostream.py --use_camera --left_camera_id 6 --right_camera_id 4 --out_dir /home/smarthc/FoundationStereo/test_outputs/output/ --invert_colormap 
"""
                # 임시 스크립트 파일 생성
                script_path = f'/tmp/foundation_stereo_{int(time.time())}.sh'
                with open(script_path, 'w') as f:
                    f.write(script)
                os.chmod(script_path, 0o755)
                cmd = ['gnome-terminal', '--', 'bash', script_path]
            elif node_name == 'tae_localization':
                cmd = ['ros2', 'run', 'local_pkg', 'tae_localization']
            else:
                self.log_message(f"❌ Unknown node: {node_name}")
                return
            
            # 프로세스 시작
            process = subprocess.Popen(cmd)
            self.processes[node_name] = process
            
            # 상태 업데이트
            self.update_status(node_name, True)
            self.log_message(f"✅ Started {node_name} (PID: {process.pid})")
            
        except Exception as e:
            self.log_message(f"❌ Failed to start {node_name}: {str(e)}")
    
    def stop_node(self, node_name):
        """노드 정지"""
        if node_name in self.processes:
            try:
                process = self.processes[node_name]
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    self.log_message(f"🔨 Force killed {node_name}")
                
                del self.processes[node_name]
                self.update_status(node_name, False)
                self.log_message(f"🛑 Stopped {node_name}")
                
            except Exception as e:
                self.log_message(f"❌ Error stopping {node_name}: {e}")
        else:
            self.log_message(f"⚠️ {node_name} is not running")
    
    def start_all_nodes(self):
        """모든 노드 시작"""
        nodes = ['tae_localization', 'glim', 'google_earth', 'foundation_stereo']
        for node in nodes:
            self.start_node(node)
            time.sleep(1)
    
    def stop_all_nodes(self):
        """모든 노드 정지"""
        for node in list(self.processes.keys()):
            self.stop_node(node)
    
    def update_status(self, node_name, is_running):
        """상태 표시 업데이트"""
        if node_name in self.status_labels:
            label = self.status_labels[node_name]
            if is_running:
                label.config(text="● Running", fg='green')
            else:
                label.config(text="● Stopped", fg='red')
        
        # 프로세스 카운트 업데이트
        count = len(self.processes)
        self.process_count_label.config(text=f"Running processes: {count}")
    
    def log_message(self, message):
        """로그 메시지 추가"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
        print(message)  # 터미널에도 출력
    
    def on_closing(self):
        """창 닫기 처리"""
        print("Closing application...")
        
        # 모든 프로세스 정지
        for node_name in list(self.processes.keys()):
            self.stop_node(node_name)
        
        self.root.quit()
        self.root.destroy()

class LocalizationUINode(Node):
    """ROS2 노드로 데이터 수신"""
    
    def __init__(self, ui):
        super().__init__('localization_ui_node')
        self.ui = ui
        
        # 토픽 구독
        self.utm_subscription = self.create_subscription(
            PointStamped,
            '/Local/utm',
            self.utm_callback,
            10
        )
        
        self.heading_subscription = self.create_subscription(
            Float64,
            'Local/heading',
            self.heading_callback,
            10
        )
        
        # 주기적 업데이트 타이머
        self.create_timer(1.0, self.update_ui)
        
        self.get_logger().info("Localization UI Node started")
    
    def utm_callback(self, msg):
        """UTM 위치 데이터 수신"""
        current_time = time.time()
        
        # UI 데이터 업데이트
        self.ui.utm_data['x'].append(msg.point.x)
        self.ui.utm_data['y'].append(msg.point.y)
        self.ui.utm_data['time'].append(current_time)
        
        self.ui.current_position['x'] = msg.point.x
        self.ui.current_position['y'] = msg.point.y
        
        # 플롯 업데이트
        if hasattr(self.ui, 'canvas'):
            self.ui.root.after_idle(self.ui.update_utm_plot)
    
    def heading_callback(self, msg):
        """헤딩 데이터 수신"""
        self.ui.current_heading = msg.data
        self.ui.heading_data.append(msg.data)
    
    def update_ui(self):
        """UI 정보 주기적 업데이트"""
        if hasattr(self.ui, 'system_text'):
            self.ui.root.after_idle(self.ui.update_system_info)

def main(args=None):
    print("Starting Localization UI with Real-time Visualization...")
    
    try:
        # UI 생성
        ui = LocalizationUI()
        root = ui.setup_ui()
        
        if root is None:
            print("Failed to create UI")
            return
        
        # ROS2 초기화 및 노드 생성 (별도 스레드)
        def ros_thread():
            try:
                rclpy.init(args=args)
                ui_node = LocalizationUINode(ui)
                rclpy.spin(ui_node)
            except Exception as e:
                print(f"ROS thread error: {e}")
            finally:
                try:
                    if rclpy.ok():
                        rclpy.shutdown()
                except:
                    pass
        
        # ROS2를 별도 스레드에서 실행
        ros_thread_obj = threading.Thread(target=ros_thread, daemon=True)
        ros_thread_obj.start()
        
        print("UI is ready. Starting main loop...")
        root.mainloop()
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
