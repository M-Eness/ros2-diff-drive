"""
Teknofest Robotaksi - Statik Engel Algilama Node'u
===================================================
Velodyne VLP-16 nokta bulutu -> zemin cikarma -> kumeleme -> costmap

Pipeline:
  /velodyne_points (PointCloud2)
      |
      v
  [Zemin Cikarma - RANSAC]
      |
      v
  [Voxel Grid Filtreleme]
      |
      v
  [Euclidean Cluster Extraction]
      |
      v
  /obstacles/bounding_boxes  (visualization_msgs/MarkerArray)
  /obstacles/centroids        (geometry_msgs/PoseArray)
  /obstacles/laser_scan       (sensor_msgs/LaserScan)  <- nav2 costmap icin
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration

import sensor_msgs_py.point_cloud2 as pc2


class StaticObstacleDetector(Node):
    """
    VLP-16 nokta bulutundan statik engelleri tespit eder.

    Adimlar:
      1. Ham bulutu al
      2. ROI (ilgi bolgesini) filtrele - aracin yakin cevresini al
      3. Zemin duslemi cikar (RANSAC benzeri yukseklik filtresi)
      4. Voxel grid ile seyrekletir
      5. Euclidean kumeleme ile nesneleri ayir
      6. Her kume icin bounding box hesapla
      7. LaserScan olarak yayinla (nav2 obstacle_layer icin)
    """

    def __init__(self):
        super().__init__('static_obstacle_detector')

        # --- Parametreler ---
        self.declare_parameter('lidar_topic', '/velodyne_points')
        self.declare_parameter('base_frame', 'base_link')

        # ROI - aracin ne kadar onunu/yanini tara (metre)
        self.declare_parameter('roi_x_min', -1.0)   # arkasi
        self.declare_parameter('roi_x_max', 15.0)   # onunu
        self.declare_parameter('roi_y_min', -5.0)   # solu
        self.declare_parameter('roi_y_max', 5.0)    # sagi
        self.declare_parameter('roi_z_min', -2.0)   # alti
        self.declare_parameter('roi_z_max', 3.0)    # ustu

        # Zemin cikarma: bu yuksekligin altindaki noktalar zemin sayilir
        self.declare_parameter('ground_z_threshold', -0.3)
        # Engel yuksekligi: bu yuksekligin uzerindekiler engel sayilir
        self.declare_parameter('obstacle_z_min', -0.2)
        self.declare_parameter('obstacle_z_max', 2.5)

        # Voxel grid boyutu (metre) - daha buyuk = daha az nokta, daha hizli
        self.declare_parameter('voxel_size', 0.1)

        # Euclidean kumeleme
        self.declare_parameter('cluster_tolerance', 0.4)  # metre
        self.declare_parameter('min_cluster_size', 5)
        self.declare_parameter('max_cluster_size', 5000)

        # Minimum engel boyutu (gida vs. gercek engel ayirimi)
        self.declare_parameter('min_obstacle_width', 0.1)   # metre
        self.declare_parameter('max_obstacle_width', 8.0)   # metre

        # LaserScan parametreleri (nav2 icin)
        self.declare_parameter('laser_scan_height_min', -0.1)
        self.declare_parameter('laser_scan_height_max', 1.5)
        self.declare_parameter('laser_angle_min', -3.14159)
        self.declare_parameter('laser_angle_max', 3.14159)
        self.declare_parameter('laser_angle_increment', 0.00872)  # 0.5 derece
        self.declare_parameter('laser_range_min', 0.3)
        self.declare_parameter('laser_range_max', 20.0)

        self._load_params()

        # QoS - sensor verisi icin BEST_EFFORT yeterli
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            sensor_qos
        )

        # Publisher'lar
        self.marker_pub = self.create_publisher(
            MarkerArray, '/obstacles/bounding_boxes', 10)
        self.pose_pub = self.create_publisher(
            PoseArray, '/obstacles/centroids', 10)
        self.scan_pub = self.create_publisher(
            LaserScan, '/obstacles/laser_scan', 10)

        self.frame_count = 0
        self.get_logger().info(
            f'StaticObstacleDetector baslatildi. '
            f'LiDAR topic: {self.lidar_topic}'
        )

    def _load_params(self):
        """Parametreleri yukle."""
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.base_frame = self.get_parameter('base_frame').value

        self.roi_x_min = self.get_parameter('roi_x_min').value
        self.roi_x_max = self.get_parameter('roi_x_max').value
        self.roi_y_min = self.get_parameter('roi_y_min').value
        self.roi_y_max = self.get_parameter('roi_y_max').value
        self.roi_z_min = self.get_parameter('roi_z_min').value
        self.roi_z_max = self.get_parameter('roi_z_max').value

        self.ground_z_thresh = self.get_parameter('ground_z_threshold').value
        self.obs_z_min = self.get_parameter('obstacle_z_min').value
        self.obs_z_max = self.get_parameter('obstacle_z_max').value

        self.voxel_size = self.get_parameter('voxel_size').value

        self.cluster_tol = self.get_parameter('cluster_tolerance').value
        self.min_cluster = self.get_parameter('min_cluster_size').value
        self.max_cluster = self.get_parameter('max_cluster_size').value

        self.min_obs_width = self.get_parameter('min_obstacle_width').value
        self.max_obs_width = self.get_parameter('max_obstacle_width').value

        self.scan_h_min = self.get_parameter('laser_scan_height_min').value
        self.scan_h_max = self.get_parameter('laser_scan_height_max').value
        self.scan_angle_min = self.get_parameter('laser_angle_min').value
        self.scan_angle_max = self.get_parameter('laser_angle_max').value
        self.scan_angle_inc = self.get_parameter('laser_angle_increment').value
        self.scan_range_min = self.get_parameter('laser_range_min').value
        self.scan_range_max = self.get_parameter('laser_range_max').value

    # ------------------------------------------------------------------
    # Ana callback
    # ------------------------------------------------------------------

    def lidar_callback(self, msg: PointCloud2):
        """Her LiDAR frame'i icin engel algilama pipeline'ini calistir."""
        self.frame_count += 1

        # 1. Nokta bulutunu numpy array'e cevir
        points = self._pointcloud2_to_numpy(msg)
        if points is None or len(points) == 0:
            return

        # 2. ROI filtresi
        points = self._roi_filter(points)
        if len(points) < 10:
            return

        # 3. Zemin cikar
        obstacle_points = self._remove_ground(points)
        if len(obstacle_points) < 5:
            self._publish_empty_scan(msg.header)
            return

        # 4. Voxel grid seyrekletme
        obstacle_points = self._voxel_downsample(obstacle_points)

        # 5. Euclidean kumeleme
        clusters = self._euclidean_clustering(obstacle_points)

        # 6. Geri bildirim yayinla
        if clusters:
            self._publish_markers(clusters, msg.header)
            self._publish_poses(clusters, msg.header)

        # 7. LaserScan yayinla (nav2 icin - her frame)
        self._publish_laser_scan(obstacle_points, msg.header)

        # Her 100 frame'de bir istatistik yazdir
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: '
                f'{len(obstacle_points)} nokta, '
                f'{len(clusters)} engel'
            )

    # ------------------------------------------------------------------
    # Adim 1: PointCloud2 -> numpy
    # ------------------------------------------------------------------

    def _pointcloud2_to_numpy(self, msg: PointCloud2):
        """PointCloud2 mesajini (N, 3) numpy array'e cevir."""
        try:
            points_list = []
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                     skip_nans=True):
                points_list.append([p[0], p[1], p[2]])

            if not points_list:
                return None
            return np.array(points_list, dtype=np.float32)
        except Exception as e:
            self.get_logger().warn(f'Nokta bulutu okuma hatasi: {e}')
            return None

    # ------------------------------------------------------------------
    # Adim 2: ROI Filtresi
    # ------------------------------------------------------------------

    def _roi_filter(self, points: np.ndarray) -> np.ndarray:
        """
        Aracin ilgi bolgesini (ROI) filtrele.
        Aracin cok uzagindaki veya arka taraftaki noktalari at.
        """
        mask = (
            (points[:, 0] > self.roi_x_min) &
            (points[:, 0] < self.roi_x_max) &
            (points[:, 1] > self.roi_y_min) &
            (points[:, 1] < self.roi_y_max) &
            (points[:, 2] > self.roi_z_min) &
            (points[:, 2] < self.roi_z_max)
        )
        return points[mask]

    # ------------------------------------------------------------------
    # Adim 3: Zemin Cikarma
    # ------------------------------------------------------------------

    def _remove_ground(self, points: np.ndarray) -> np.ndarray:
        """
        Zemin noktalarini cikar.

        Yaklasim: Yukseklik esigine dayali basit filtre.
        VLP-16 arac uzerinde ~620mm yukseklikte, zemin sensore gore
        yaklasik -0.6m civarinda. ground_z_threshold bu mesafeyi ayarlar.

        Daha gelismis: pcl RANSAC plane segmentation (C++ node) kullanilabilir.
        """
        # Once sadece zemin yuksekligindeki noktalari bul
        ground_mask = points[:, 2] < self.ground_z_thresh
        non_ground = points[~ground_mask]

        # Engel yukseklik araligini da filtrele
        obstacle_mask = (
            (non_ground[:, 2] > self.obs_z_min) &
            (non_ground[:, 2] < self.obs_z_max)
        )
        return non_ground[obstacle_mask]

    # ------------------------------------------------------------------
    # Adim 4: Voxel Grid Seyrekletme
    # ------------------------------------------------------------------

    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """
        Voxel grid ile nokta bulutunu seyreklet.
        Her voxel icindeki noktalarin merkezini al.
        """
        if len(points) == 0:
            return points

        voxel_size = self.voxel_size
        # Voxel indekslerini hesapla
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)

        # Her benzersiz voxel icin merkezi hesapla
        unique_voxels, inverse = np.unique(
            voxel_indices, axis=0, return_inverse=True)

        downsampled = np.zeros((len(unique_voxels), 3), dtype=np.float32)
        counts = np.zeros(len(unique_voxels), dtype=np.int32)

        for i, idx in enumerate(inverse):
            downsampled[idx] += points[i]
            counts[idx] += 1

        counts = counts.reshape(-1, 1)
        downsampled = downsampled / counts

        return downsampled

    # ------------------------------------------------------------------
    # Adim 5: Euclidean Kumeleme
    # ------------------------------------------------------------------

    def _euclidean_clustering(self, points: np.ndarray) -> list:
        """
        Basit Euclidean kumeleme (KD-tree olmadan, numpy ile).

        Buyuk sahnelerde pcl C++ kullanmak daha hizlidir.
        Bu implementasyon yarisma parkuru boyutlari icin yeterlidir.

        Returns:
            clusters: Her eleman bir (N, 3) numpy array (bir engel)
        """
        if len(points) == 0:
            return []

        tol_sq = self.cluster_tol ** 2
        n = len(points)
        visited = np.zeros(n, dtype=bool)
        clusters = []

        for i in range(n):
            if visited[i]:
                continue

            # BFS ile kumeyi genislet
            cluster_indices = [i]
            queue = [i]
            visited[i] = True

            while queue:
                curr = queue.pop(0)
                # Tum noktalara mesafe hesapla (vektorize)
                diffs = points - points[curr]
                dist_sq = np.sum(diffs ** 2, axis=1)
                neighbors = np.where(
                    (dist_sq < tol_sq) & (~visited)
                )[0]

                for nb in neighbors:
                    visited[nb] = True
                    cluster_indices.append(nb)
                    queue.append(nb)

            # Boyut kontrolu
            size = len(cluster_indices)
            if self.min_cluster <= size <= self.max_cluster:
                cluster_pts = points[cluster_indices]
                # Genislik kontrolu
                width_x = np.max(cluster_pts[:, 0]) - np.min(cluster_pts[:, 0])
                width_y = np.max(cluster_pts[:, 1]) - np.min(cluster_pts[:, 1])
                max_width = max(width_x, width_y)
                if self.min_obs_width <= max_width <= self.max_obs_width:
                    clusters.append(cluster_pts)

        return clusters

    # ------------------------------------------------------------------
    # Adim 6a: Marker yayini (RViz goruntuleme)
    # ------------------------------------------------------------------

    def _publish_markers(self, clusters: list, header):
        """Her kume icin bounding box marker yayinla (RViz icin)."""
        marker_array = MarkerArray()

        # Once onceki marker'lari temizle
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.ns = 'obstacles'
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, cluster in enumerate(clusters):
            min_pt = np.min(cluster, axis=0)
            max_pt = np.max(cluster, axis=0)
            center = (min_pt + max_pt) / 2.0
            size = max_pt - min_pt

            marker = Marker()
            marker.header = header
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.w = 1.0

            # Minimum gorunur boyut
            marker.scale.x = max(float(size[0]), 0.1)
            marker.scale.y = max(float(size[1]), 0.1)
            marker.scale.z = max(float(size[2]), 0.1)

            # Renk: kirmizi (engel)
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.0
            marker.color.a = 0.7

            marker.lifetime = Duration(sec=0, nanosec=200_000_000)  # 0.2s
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    # ------------------------------------------------------------------
    # Adim 6b: Pose yayini
    # ------------------------------------------------------------------

    def _publish_poses(self, clusters: list, header):
        """Her engelin merkezini PoseArray olarak yayinla."""
        pose_array = PoseArray()
        pose_array.header = header

        for cluster in clusters:
            centroid = np.mean(cluster, axis=0)
            pose = Pose()
            pose.position.x = float(centroid[0])
            pose.position.y = float(centroid[1])
            pose.position.z = float(centroid[2])
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.pose_pub.publish(pose_array)

    # ------------------------------------------------------------------
    # Adim 7: LaserScan yayini (nav2 obstacle_layer icin)
    # ------------------------------------------------------------------

    def _publish_laser_scan(self, obstacle_points: np.ndarray, header):
        """
        3D nokta bulutunu 2D LaserScan'e donustur.
        nav2'nin obstacle_layer'i bu formati dogrudan tuketerek
        costmap'i guncelleyebilir.

        Belirli yukseklik araligindaki (scan_h_min - scan_h_max) noktalar
        kullanilir.
        """
        scan = LaserScan()
        scan.header = header
        scan.header.frame_id = self.base_frame

        num_bins = int(
            (self.scan_angle_max - self.scan_angle_min) / self.scan_angle_inc
        )

        scan.angle_min = self.scan_angle_min
        scan.angle_max = self.scan_angle_max
        scan.angle_increment = self.scan_angle_inc
        scan.range_min = self.scan_range_min
        scan.range_max = self.scan_range_max
        scan.time_increment = 0.0
        scan.scan_time = 0.1

        ranges = np.full(num_bins, self.scan_range_max + 1.0)

        if len(obstacle_points) > 0:
            # Yalnizca belirlenen yukseklik araligindaki noktalar
            height_mask = (
                (obstacle_points[:, 2] >= self.scan_h_min) &
                (obstacle_points[:, 2] <= self.scan_h_max)
            )
            scan_pts = obstacle_points[height_mask]

            if len(scan_pts) > 0:
                # Her nokta icin aci ve mesafe hesapla
                angles = np.arctan2(scan_pts[:, 1], scan_pts[:, 0])
                distances = np.sqrt(
                    scan_pts[:, 0] ** 2 + scan_pts[:, 1] ** 2
                )

                # Gecerli mesafe araligindaki noktalar
                valid = (
                    (distances >= self.scan_range_min) &
                    (distances <= self.scan_range_max)
                )
                angles = angles[valid]
                distances = distances[valid]

                # Aci indekslerine donustur ve en yakin mesafeyi al
                indices = ((angles - self.scan_angle_min) /
                           self.scan_angle_inc).astype(int)
                valid_idx = (indices >= 0) & (indices < num_bins)
                indices = indices[valid_idx]
                distances = distances[valid_idx]

                for idx, dist in zip(indices, distances):
                    if dist < ranges[idx]:
                        ranges[idx] = dist

        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)

    def _publish_empty_scan(self, header):
        """Hic engel yoksa bos scan yayinla."""
        scan = LaserScan()
        scan.header = header
        scan.header.frame_id = self.base_frame
        num_bins = int(
            (self.scan_angle_max - self.scan_angle_min) / self.scan_angle_inc
        )
        scan.angle_min = self.scan_angle_min
        scan.angle_max = self.scan_angle_max
        scan.angle_increment = self.scan_angle_inc
        scan.range_min = self.scan_range_min
        scan.range_max = self.scan_range_max
        scan.ranges = [self.scan_range_max + 1.0] * num_bins
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = StaticObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
