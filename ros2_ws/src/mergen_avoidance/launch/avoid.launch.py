"""Gazebo + TurtleBot3 + engelden sakinma -- tek komutla tam yigin.

    ros2 launch mergen_avoidance avoid.launch.py world:=w2_cone_slalom

Argumanlar:
    world       dunya adi (worlds/ altindaki dosya adi, .world uzantisiz)
    gui         Gazebo penceresi acilsin mi (toplu test icin false)
    rviz        RViz acilsin mi (sensor + karar gorselleri)
    metrics     CSV metrik kaydi tutulsun mu
    pedestrian  dinamik engel surucusu calissin mi (w8 icin otomatik)
    x_pose/y_pose/yaw  robotun baslangic konumu
    params_file ayar dosyasi (config/avoider.yaml)
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("mergen_avoidance")
    gazebo_share = get_package_share_directory("gazebo_ros")

    # TurtleBot3 modeli: waffle govdesi burger'dan genis, yarisma aracina
    # (120-180 cm en) oranla daha gercekci bir sakinma problemi uretir.
    tb3_model = os.environ.get("TURTLEBOT3_MODEL", "waffle")
    if tb3_model not in ("burger", "waffle", "waffle_pi"):
        raise RuntimeError(
            f"TURTLEBOT3_MODEL gecersiz: '{tb3_model}' "
            "(burger | waffle | waffle_pi olmali)"
        )

    # TurtleBot3 paketlerini ARAMADAN once kontrol et.
    # Neden: `get_package_share_directory` eksik pakette PackageNotFoundError
    # atar ve ros2launch bunu "InvalidLaunchFileError: may have a syntax error"
    # icine sarar -- yani kurulum eksikligi, KOD HATASI gibi gorunur. Bir
    # defa yasandi; mesaji burada net tutuyoruz.
    def _share(pkg: str) -> str:
        try:
            return get_package_share_directory(pkg)
        except PackageNotFoundError as exc:
            raise RuntimeError(
                f"\n\n'{pkg}' paketi kurulu degil.\n"
                "TurtleBot3 simulasyon paketleri eksik. Cozum:\n\n"
                "    sudo bash install/02_turtlebot3_sim.sh\n"
                "    bash install/03_env_and_build.sh\n"
            ) from exc

    urdf_path = os.path.join(
        _share("turtlebot3_description"), "urdf", f"turtlebot3_{tb3_model}.urdf")
    sdf_path = os.path.join(
        _share("turtlebot3_gazebo"), "models", f"turtlebot3_{tb3_model}", "model.sdf")

    for path, what in ((urdf_path, "URDF"), (sdf_path, "Gazebo SDF")):
        if not os.path.exists(path):
            raise RuntimeError(
                f"\n\nTurtleBot3 {what} dosyasi bulunamadi:\n  {path}\n"
                f"Paket kurulu ama '{tb3_model}' modeli yok.\n"
                "TURTLEBOT3_MODEL degerini kontrol et (burger | waffle | waffle_pi).\n"
            )
    with open(urdf_path, "r", encoding="utf8") as fh:
        robot_description = fh.read()

    # ---- argumanlar ----
    args = [
        DeclareLaunchArgument("world", default_value="w2_cone_slalom",
                              description="worlds/ altindaki dunya adi (.world'suz)"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("metrics", default_value="true"),
        DeclareLaunchArgument("avoider", default_value="true",
                              description="false ise sakinma dugumu calismaz; "
                                          "robot durur. Lidar verisini incelemek icin."),
        DeclareLaunchArgument("pedestrian", default_value="auto",
                              description="true | false | auto (w8 ise acilir)"),
        DeclareLaunchArgument("x_pose", default_value="0.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(pkg_share, "config", "avoider.yaml")),
        # Varsayilan 17 m: tum parkurlarimiz 9-18 m uzunlugunda, yani arac
        # gorevini bitirip DURUYOR. 0 verirsen hedef kapanir ve arac
        # engeller bittikten sonra sonsuza kadar ilerler.
        DeclareLaunchArgument("goal_x", default_value="17.0",
                              description="bitis noktasi [m]; 0 = kapali"),
        DeclareLaunchArgument("output_dir",
                              default_value=os.path.expanduser("~/mergen_results")),
    ]

    world = LaunchConfiguration("world")
    world_path = PathJoinSubstitution([pkg_share, "worlds",
                                       PythonExpression(["'", world, "' + '.world'"])])

    # "auto": dunya adi w8 ile basliyorsa yaya surucusunu calistir.
    pedestrian_on = PythonExpression([
        "'", LaunchConfiguration("pedestrian"), "' == 'true' or ",
        "('", LaunchConfiguration("pedestrian"), "' == 'auto' and '",
        world, "'.startswith('w8'))",
    ])

    # gzserver include'u KAPSAMLANMALI (GroupAction, scoped=True).
    #
    # Neden: `launch_arguments={"world": world_path}` verdigimizde, gzserver'in
    # kendi `world` argumani UST KAPSAMA sizip bizim `world` yapilandirmamizi
    # tam dosya yoluyla eziyor. Sonucu ilk Gazebo kosusunda olculdu: metrik
    # dugumune `scenario` olarak "w2_cone_slalom" yerine
    # ".../worlds/w2_cone_slalom.world" gitti ve CSV yanlis dizine yazildi.
    # GroupAction, include'un yapilandirmalarini kendi icinde tutar.
    gzserver = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, "launch", "gzserver.launch.py")),
            launch_arguments={"world": world_path, "verbose": "false"}.items(),
        ),
    ], scoped=True, forwarding=True)

    gzclient = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, "launch", "gzclient.launch.py")),
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
    ], scoped=True, forwarding=True)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_turtlebot3",
        output="screen",
        arguments=[
            "-entity", tb3_model,
            "-file", sdf_path,
            "-x", LaunchConfiguration("x_pose"),
            "-y", LaunchConfiguration("y_pose"),
            "-z", "0.01",
            "-Y", LaunchConfiguration("yaw"),
        ],
    )

    avoider = Node(
        package="mergen_avoidance",
        executable="avoider",
        name="mergen_avoider",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("avoider")),
        parameters=[LaunchConfiguration("params_file"),
                    {"use_sim_time": True,
                     # TIPI ACIKCA BELIRT. Aksi halde launch bu degeri METIN
                     # olarak gecirir ve dugum "double bekliyordum, string
                     # geldi" diye acilista coker. Ayni tuzaga `axis: "y"`
                     # ile bir kez dusuldu (YAML onu boolean yapmisti).
                     "goal_x": ParameterValue(LaunchConfiguration("goal_x"),
                                              value_type=float)}],
    )

    metrics = Node(
        package="mergen_avoidance",
        executable="metrics_logger",
        name="mergen_metrics_logger",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("metrics")),
        parameters=[{
            "use_sim_time": True,
            "scenario": world,
            "output_dir": LaunchConfiguration("output_dir"),
        }],
    )

    pedestrian = Node(
        package="mergen_avoidance",
        executable="pedestrian",
        name="mergen_pedestrian",
        namespace="pedestrian",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(pedestrian_on),
        parameters=[{
            "use_sim_time": True,
            "mode": "triggered",
            # Tetik mesafesi HESAPLA, tahmin etme.
            # Yaya x=4.0'da, y=-2.0'dan 0.45 m/s ile karsiya geciyor; serit
            # ortasina varmasi 4.4 s suruyor. Robot 0.22 m/s.
            #   trigger_x=3.2 -> robot x=4.0'a varinca yaya y=-0.36, yani
            #     TAM ONUNDE: kacinilmaz carpisma, adil bir test degil.
            #   trigger_x=1.8 -> yaya robotun ONUNDEN gecer, robot ~1.5 m
            #     mesafeden gorup yavaslar/durur, yaya gecince devam eder.
            # Ikincisi sartnamedeki "engel tamamen gecene kadar bekledi"
            # davranisini olcen kurgu.
            "trigger_x": 1.8,
            "speed": 0.45,
            # "y" YAZMA: YAML 1.1 ciplak 'y'yi boolean'a cevirir ve dugum coker.
            # Bkz. pedestrian_node._normalize_axis
            "axis": "lateral",
            "travel": 4.0,
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", os.path.join(pkg_share, "rviz", "mergen.rviz")],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(args + [
        LogInfo(msg=["mergen-01 | dunya: ", world, " | model: " + tb3_model]),
        gzserver, gzclient,
        robot_state_publisher, spawn,
        avoider, metrics, pedestrian, rviz,
    ])
