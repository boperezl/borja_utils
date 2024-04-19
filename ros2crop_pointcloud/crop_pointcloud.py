import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
#import sensor_msgs.point_cloud2 as pc2
import numpy as np
import struct

class PointCloudCropping(Node):
    def __init__(self):
        super().__init__('point_cloud_cropping')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/maria/depth/color/points',
            self.point_cloud_callback,
            10)
        self.publisher_ = self.create_publisher(PointCloud2, '/movvo/cropped_points', 10)

    def point_cloud_callback(self, msg):
        # Aquí maneja la nube de puntos recibida
        # msg es el mensaje de nube de puntos
        # Realiza el recorte según las coordenadas proporcionadas
        # Por ejemplo, podrías usar numpy para filtrar puntos dentro del bounding box
        
        # Supongamos que las coordenadas del bounding box son (x1, y1, z1) y (x2, y2, z2)
        x1, y1 = -0.5, -1  # coordenadas de la esquina opuesta 1 del bounding box
        x2, y2 = 0.5, 0.5    # coordenadas de la esquina opuesta 2 del bounding box
        
        cropped_points = []
        
        data = msg.data
        point_step = msg.point_step
        row_step = msg.row_step
        fields = msg.fields
        h = msg.height
        w = msg.width
        MAX_Z = -999999.999
        for row in range(0,h,4):
            for col in range(0,w,4):
                # Calcular el índice del punto en los datos
                index = row * row_step + col * point_step

                # Extraer las coordenadas (x, y, z) del punto
                x, y, z = None, None, None
                for field in fields:
                    if field.name == 'x':
                        x = struct.unpack_from('f', data, index + field.offset)[0]
                    elif field.name == 'y':
                        y = struct.unpack_from('f', data, index + field.offset)[0]
                    elif field.name == 'z':
                        z = struct.unpack_from('f', data, index + field.offset)[0]

                # if x is not None and y is not None and z is not None:
                #     print("Coordenadas del punto: x={}, y={}, z={}".format(x, y, z))
                if ((x >= x1 and x <= x2) and (y >= y1 and y <= y2)):
                    if z > MAX_Z:
                        MAX_Z = z
                    cropped_points.append([x,y,z])

        print(f"MAX_Z in current pointcloud: {MAX_Z}")
        print(f"Cropped {len(cropped_points)} points")
        # for point in msg.data:
        #     x, y, z = point[:3]
        #     if x1 <= x <= x2 and y1 <= y <= y2 and z1 <= z <= z2:
        #         cropped_points.append(point)
        
        output_pc = self.pclist2pcmsg(cropped_points)

        # Crea un nuevo mensaje de nube de puntos con los puntos recortados
        #cropped_msg = PointCloud2.create_cloud(msg.header, msg.fields, output_pc)
        
        # Publica la nube de puntos recortada en el nuevo topic
        self.publisher_.publish(output_pc)

    def pclist2pcmsg(self, points):
        # Definir los campos de los puntos
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Convertir la lista de puntos a un array numpy
        points_array = np.array(points, dtype=np.float32)

        # Crear el mensaje PointCloud2
        msg = PointCloud2()
        msg.header.frame_id = 'camera_color_optical_frame'  # Cambia 'map' por el marco de referencia adecuado
        msg.height = 1
        msg.width = len(points_array)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12  # Tamaño en bytes de un punto (3 campos float32)
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points_array.tostring()

        return msg
        # Publicar el mensaje
        # self.publisher_.publish(msg)
        # self.get_logger().info('Nube de puntos publicada')

def main(args=None):
    rclpy.init(args=args)
    point_cloud_cropping = PointCloudCropping()
    rclpy.spin(point_cloud_cropping)
    point_cloud_cropping.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
