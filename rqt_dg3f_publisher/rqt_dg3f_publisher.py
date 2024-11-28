from ament_index_python import get_resource
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Bool, Int32
from rqt_gui_py.plugin import Plugin
from rqt_gui.main import Main
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import os
import sys
import math
import threading
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QDialog
from python_qt_binding.QtCore import Signal, QProcess

import matplotlib
matplotlib.use('Qt5Agg')


class JointPlot(FigureCanvas):
    def __init__(self, parent=None, title='Joint Plot'):
        # ... existing initialization code ...
        self.fig = Figure(figsize=(5, 4), dpi=100)
        super(JointPlot, self).__init__(self.fig)

        # 플롯 설정
        self.axes = self.fig.add_subplot(111)
        self.axes.set_title(title)
        self.axes.set_ylabel('Angle (deg)')
        self.axes.grid(True)

        # x축 숨기기
        self.axes.xaxis.set_visible(False)

        # 데이터 초기화
        self.data_points = 100
        self.time_data = list(range(self.data_points))
        self.joint_data = [[] for _ in range(4)]
        self.target_data = [[] for _ in range(4)]

        for i in range(4):
            self.joint_data[i] = [0] * self.data_points
            self.target_data[i] = [0] * self.data_points

        # 라인 초기화
        self.lines = []
        self.target_lines = []
        colors = ['r', 'b', 'g', 'y']
        labels = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4']

        for i in range(4):
            # 실제 joint 값을 위한 실선
            line, = self.axes.plot(self.time_data, self.joint_data[i],
                                   colors[i], label=labels[i])
            self.lines.append(line)

            # target joint 값을 위한 점선
            target_line, = self.axes.plot(self.time_data, self.target_data[i],
                                          colors[i], linestyle='--',
                                          label=f'{labels[i]} Target')
            self.target_lines.append(target_line)

        self.axes.legend(loc='upper right')
        self.axes.set_ylim([-90, 90])

        # 레이아웃 조정
        self.fig.tight_layout()
        
        self.axes.legend(
            loc='upper right',
            prop={'size': 8},  # 폰트 크기 축소
            ncol=2,  # 열 수를 2로 설정하여 가로로 배치
            bbox_to_anchor=(1, 1),  # 위치 미세 조정
            framealpha=0.8,  # 배경 투명도 설정
            frameon=True,  # 테두리 표시
        )
        # 부모 위젯에 추가
        if parent:
            layout = parent.layout()
            if layout is not None:
                layout.addWidget(self)
                
        # Add these lines after initialization
        self.draw_idle_enabled = True
        self.fig.canvas.draw()
        self.background = self.fig.canvas.copy_from_bbox(self.axes.bbox)

    def update_plot(self, new_joint_data, new_target_data):
        try:
            # Update data
            for i in range(4):
                self.joint_data[i] = self.joint_data[i][1:] + [new_joint_data[i]]
                self.lines[i].set_ydata(self.joint_data[i])
                
                self.target_data[i] = self.target_data[i][1:] + [new_target_data[i]]
                self.target_lines[i].set_ydata(self.target_data[i])
            
            # Restore background
            self.fig.canvas.restore_region(self.background)
            
            # Redraw just the lines
            for line in self.lines + self.target_lines:
                self.axes.draw_artist(line)
            
            # Update the display
            self.fig.canvas.blit(self.axes.bbox)
            self.fig.canvas.flush_events()
            
        except Exception as e:
            print(f"Error updating plot: {e}")


class RqtDelto3FPublisher(Plugin):
    joint_signal = Signal(list)

    def __init__(self, context):
        super().__init__(context)
        self._context = context

        # 변수 초기화
        self.current_joints = [0.0] * 12
        self.last_target_joint = [0.0] * 12
        self.target_joint = [0.0] * 12
        self.lock = threading.Lock()

        # 메인 위젯 설정 (QDialog 사용)
        self._widget = QDialog()
        self._widget.setObjectName('DeltoRqtPluginUi')

        # 리소스 파일 경로
        _, package_path = get_resource("packages", "rqt_dg3f_publisher")
        ui_file = os.path.join(package_path, "share",
                               "rqt_dg3f_publisher", "resource", "delto_rqt.ui")
        rviz_config = os.path.join(
            package_path, "share", "rqt_dg3f_publisher", "config", "default.rviz")

        # UI 파일 로드
        loadUi(ui_file, self._widget)

        # set
        self.setup_plots()
        
        # RViz2 프로세스 설정
        # self.rviz_process = QProcess(self._widget)
        # self.rviz_process.finished.connect(self.on_rviz_finished)

        # ROS 통신 설정
        self.setup_ros_communication()

        # 버튼 연결
        self.setup_connections()

        # 시그널 연결
        self.joint_signal.connect(self.update_joint_ui)

        # 컨텍스트에 위젯 추가
        context.add_widget(self._widget)

        # RViz2 실행
        # self.start_rviz(rviz_config)

    def __del__(self):
        self.shutdown_plugin()

    def setup_plots(self):
        # F1 Plot
        self.f1_plot = JointPlot(title='Finger 1 Joint Angles')
        self._widget.F1_plot.layout().addWidget(self.f1_plot)

        # F2 Plot
        self.f2_plot = JointPlot(title='Finger 2 Joint Angles')
        self._widget.F2_plot.layout().addWidget(self.f2_plot)

        # F3 Plot
        self.f3_plot = JointPlot(title='Finger 3 Joint Angles')
        self._widget.F3_plot.layout().addWidget(self.f3_plot)

    def start_rviz(self, config_file=None):
        cmd = 'rviz2'
        args = []
        if config_file and os.path.exists(config_file):
            args.extend(['-d', config_file])

        # Qt 창 크기와 위치 설정
        geometry = f"800x600+{self._widget.x() + self._widget.width() + 10}+{self._widget.y()}"
        args.extend(['--geometry', geometry])

        # self.rviz_process.start(cmd, args)

    def setup_plots(self):
        # F1 Plot
        self.f1_plot = JointPlot(title='Finger 1 Joint Angles')
        self._widget.F1_plot.layout().addWidget(self.f1_plot)

        # F2 Plot
        self.f2_plot = JointPlot(title='Finger 2 Joint Angles')
        self._widget.F2_plot.layout().addWidget(self.f2_plot)

        # F3 Plot
        self.f3_plot = JointPlot(title='Finger 3 Joint Angles')
        self._widget.F3_plot.layout().addWidget(self.f3_plot)

    def on_rviz_finished(self):
        print("RViz2 process finished")

    def setup_ros_communication(self):
        
        """
        Sets up the ROS communication for the gripper control.
        This method initializes various ROS publishers and a subscriber for 
        communicating with the gripper. It also sets up a timer for periodic 
        callbacks.
        Publishers:
        - target_joint_pub: Publishes target joint positions to "gripper/target_joint".
        - grasp_pub: Publishes grasp commands to "gripper/grasp".
        - idle_target_joint_pub: Publishes idle target joint positions to "gripper/idle_target_joint".
        - write_register_pub: Publishes register write commands to "gripper/write_register".
        - set_gain_publisher: Publishes gain settings to "gripper/request/gain".
        - fixed_joint_publisher: Publishes fixed joint positions to "gripper/fixed_joint".
        - load_pose_pub: Publishes load pose commands to "gripper/load_pose".
        - save_pose_pub: Publishes save pose commands to "gripper/save_pose".
        - grasp_mode_pub: Publishes grasp mode commands to "gripper/grasp_mode".
        Subscriber:
        - joint_sub: Subscribes to joint state updates from "/gripper/joint_states".
        Timer:
        - timer: Calls `self.timer_callback` at a 0.01-second interval.  
        """
        
        self.target_joint_pub = self._context.node.create_publisher(
            Float32MultiArray, "gripper/target_joint", 10)
        self.grasp_pub = self._context.node.create_publisher(
            Bool, "gripper/grasp", 10)
        self.joint_sub = self._context.node.create_subscription(
            JointState, "/gripper/joint_states", self.joint_callback, 1)
        self.idle_target_joint_pub = self._context.node.create_publisher(
            Float32MultiArray, "gripper/idle_target_joint", 10)
        self.write_register_pub = self._context.node.create_publisher(
            Int16MultiArray, "gripper/write_register", 10)
        self.set_gain_publisher = self._context.node.create_publisher(
            Int16MultiArray, "gripper/request/gain", 10)
        self.fixed_joint_publisher = self._context.node.create_publisher(
            Int16MultiArray, "gripper/fixed_joint", 10)
        self.timer = self._context.node.create_timer(0.05, self.timer_callback)

        self.load_pose_pub = self._context.node.create_publisher(
            Int32, "gripper/load_pose", 10)
        self.save_pose_pub = self._context.node.create_publisher(
            Int32, "gripper/save_pose", 10)

        self.grasp_mode_pub = self._context.node.create_publisher(
            Int32, "gripper/grasp_mode", 10)

    def setup_connections(self):
        """
        Sets up the connections between the UI elements and their respective callback functions.
        This method connects various buttons and sliders in the UI to their corresponding
        callback functions to handle user interactions.
        Connections:
        - target_joint_button: Connects to target_joint_callback.
        - grasp_button: Connects to grasp_callback.
        - ungrasp_button: Connects to ungrasp_callback.
        - load_gain_button: Connects to load_gain.
        - save_gain_button: Connects to save_gain.
        - fixed_joint_button: Connects to fixed_joint.
        - load_pose_button: Connects to load_pose_callback.
        - save_pose_button: Connects to save_pose_callback.
        - tE_grasp_mode: Connects to grasp_mode_callback on text change.
        - Sliders (slider1 to slider12): Connects to update_slider_value with the slider index.
        """
        
        self._widget.target_joint_button.clicked.connect(
            self.target_joint_callback)
        self._widget.grasp_button.clicked.connect(self.grasp_callback)
        self._widget.ungrasp_button.clicked.connect(self.ungrasp_callback)
        self._widget.load_gain_button.clicked.connect(self.load_gain)
        self._widget.save_gain_button.clicked.connect(self.save_gain)
        self._widget.fixed_joint_button.clicked.connect(self.fixed_joint)

        self._widget.load_pose_button.clicked.connect(self.load_pose_callback)
        self._widget.save_pose_button.clicked.connect(self.save_pose_callback)
        self._widget.tE_grasp_mode.textChanged.connect(
            self.grasp_mode_callback)

        for i in range(1, 13):
            slider = getattr(self._widget, f"slider{i}")
            slider.valueChanged.connect(
                lambda value, index=i: self.update_slider_value(index))

    def load_pose_callback(self):
        msg = Int32()
        msg.data = int(self._widget.tE_pose.toPlainText())
        self.load_pose_pub.publish(msg)

    def save_pose_callback(self):
        msg = Int32()
        msg.data = int(self._widget.tE_pose.toPlainText())
        self.save_pose_pub.publish(msg)

    def grasp_mode_callback(self):
        msg = Int32()
        msg.data = int(self._widget.tE_grasp_mode.toPlainText())
        self.grasp_mode_pub.publish(msg)

    def update_slider_value(self, index):
        with self.lock:
            slider_index = index - 1
            self.target_joint[slider_index] = getattr(
                self._widget, f"slider{index}").value()

            for i in range(12):
                field_name = f"tE_F{int(i/4)+1}M{(i %4)+1}"
                value = str(round(self.target_joint[i]))
                getattr(self._widget, field_name).setText(value)

        self.target_joint_callback()

    def timer_callback(self):
        msg = Float32MultiArray()
        with self.lock:
            msg.data = self.last_target_joint
            self.idle_target_joint_pub.publish(msg)

    def joint_callback(self, msg):
        current_joints = [round(i * 180.0 / math.pi, 2) for i in msg.position]
        self.joint_signal.emit(current_joints)

    def update_joint_ui(self, current_joints):
        try:
            self.current_joints = current_joints
            for i in range(12):
                getattr(
                    self._widget, f"tE_F{int(i/4)+1}M{(i %4)+1}_2"
                ).setText(str(round(self.current_joints[i], 2)))

        # Update plots with error handling
            try:
                self.f1_plot.update_plot(current_joints[0:4], self.target_joint[0:4])
                self.f2_plot.update_plot(current_joints[4:8], self.target_joint[4:8])
                self.f3_plot.update_plot(current_joints[8:12], self.target_joint[8:12])
            except Exception as e:
                print(f"Error updating plots: {e}")
            
        except Exception as e:
            print(f"Error in update_joint_ui: {e}")
        
    def target_joint_callback(self):
        
        joint = []
        joint_deg = []

        for i in range(1, 4):
            for j in range(1, 5):
                data_str = getattr(self._widget, f"tE_F{i}M{j}").toPlainText()
                data = float(data_str)
                joint_deg.append(data)
                joint.append(data * math.pi / 180.0)

        msg = Float32MultiArray()
        with self.lock:
            self.last_target_joint = joint
            self.target_joint = joint_deg

        msg.data = joint
        self.target_joint_pub.publish(msg)

    def fixed_joint(self):
        msg = Int16MultiArray()
        data = []
        for i in range(12):
            fixed_joint = getattr(
                self._widget, f"fixed_joint_{i+1}").isChecked()
            data.append(int(fixed_joint))

        msg.data = data
        self.fixed_joint_publisher.publish(msg)

    def load_gain(self):
        pass  # TODO: Implement gain loading

    def save_gain(self):
        msg = Int16MultiArray()
        data = []
        for i in range(12):
            pgain = getattr(self._widget, f"pgain_{i+1}").toPlainText()
            data.append(int(pgain))

        for i in range(12):
            dgain = getattr(self._widget, f"dgain_{i+1}").toPlainText()
            data.append(int(dgain))

        msg.data = data
        self.set_gain_publisher.publish(msg)

    def grasp_callback(self):
        msg = Bool()
        msg.data = True
        self.grasp_pub.publish(msg)

    def ungrasp_callback(self):
        msg = Bool()
        msg.data = False
        self.grasp_pub.publish(msg)

    def shutdown_plugin(self):

        # if self.rviz_process.state() == QProcess.Running:
        # self.rviz_process.terminate()
        # self.rviz_process.waitForFinished(1000)

        if hasattr(self, 'timer'):
            self.timer.cancel()


def main():
    sys.exit(Main().main(
        sys.argv, standalone="rqt_dg3f_publisher.rqt_dg3f_publisher"))


if __name__ == '__main__':
    main()
