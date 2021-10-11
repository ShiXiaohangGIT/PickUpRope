import super_voxel
import numpy as np
import open3d as o3d
import time
import copy
import math
import get_point_cloud

# 基于超体素包围盒来寻找相邻超体素的方法

# ******************** 图类 
class graph:
    def __init__(self,vertex_number):
        # 储存与节点相连的其他点
        self.vertex = [[] for i in range(vertex_number)]
        # 储存边
        self.edge = np.zeros((vertex_number,vertex_number,3))
        # 储存节点坐标
        self.vertex_xyz = [[] for i in range(vertex_number)]
        # 储存节点代表的超体素所包含的点的数量
        self.vertex_length = []
        # 按照超体素标签储存点云
        self.super_voxel_label_points_list = [[] for i in range(n_super_voxels)]
        # 储存节点梯度
        self.vertex_degree = []

# ******************** 轴类 
class axis:
    def __init__(self,AxisNumber):
        # 储存与节点相连的其他点
        self.vertex = []
        # 储存节点梯度
        self.vertex_degree = []
        # 储存末端坐标
        self.terminal = []
        # 储存末端编号
        self.terminal_index = []
        # 储存末端向量
        self.vector = []

# ******************** 被连接起来的轴类 
class connect_axis:
    def __init__(self,connect_axis_list, connected_vertex_list):
        self.axis = connect_axis_list
        self.axis_length_list = []
        self.connected_vertex = connected_vertex_list
    
    # 计算每根轴线的得分
    def score(self, occlusion_axis_list):
        k_list = [1,0.5,0.25,0.125,0,0,0,0]
        k0 = 1
        k1 = 0.5
        for i in range(len(self.axis)):
            single_edge_length = calculate(self.axis[i])
            self.axis_length_list.append( k_list[len(occlusion_axis_list[i])] * single_edge_length )
        return self.axis_length_list

# ******************** 计算每根轴线的长度
def calculate(single_axis):
    axis_length_sum = 0
    for i in range(len(single_axis)-1):
        # 对于第一第二节点组成的边，将边的长度直接加入总长度中
        if i == 0:
            length_edge = np.linalg.norm(
                    copy.deepcopy(
                        super_voxel_graph.edge[single_axis[0], single_axis[1], :]
                )
            )
            axis_length_sum = axis_length_sum + length_edge
            continue
        
        # 对于第二节点之后的边，计算其在上一条边方向上的投影长度，将投影长度加入总长度中
        vector_edge_0 = copy.deepcopy(super_voxel_graph.edge[single_axis[i-1], single_axis[i], :])
        vector_edge_1 = copy.deepcopy(super_voxel_graph.edge[single_axis[i], single_axis[i+1], :])
        length_edge = np.dot(vector_edge_0, vector_edge_1) / np.linalg.norm( vector_edge_0 )

        # 总长度
        axis_length_sum = axis_length_sum + length_edge

    return axis_length_sum

# ******************** 计算每根轴线的一半长度处的节点
def calculate_half(single_axis,total_length):
    # 多数代码与calculate函数相同
    axis_length_sum = 0
    for i in range(len(single_axis)-1):
        if i == 0:
            length_edge = np.linalg.norm(
                    copy.deepcopy(
                        super_voxel_graph.edge[single_axis[0], single_axis[1], :]
                )
            )
            axis_length_sum = axis_length_sum + length_edge
            if axis_length_sum > total_length/2:
                return [single_axis[0], single_axis[1]]
            continue

        vector_edge_0 = copy.deepcopy(super_voxel_graph.edge[single_axis[i-1], single_axis[i], :])
        vector_edge_1 = copy.deepcopy(super_voxel_graph.edge[single_axis[i], single_axis[i+1], :])
        length_edge = np.dot(vector_edge_0, vector_edge_1) / np.linalg.norm( vector_edge_0 )

        axis_length_sum = axis_length_sum + length_edge

        # 统计的长度已至全长的一半，则输出对应的节点
        if axis_length_sum > total_length/2:
            return [single_axis[i], single_axis[i+1]]

# ******************** 显示连接线 
def line_show(line_list,point_list,color_list,others_shown):

    # 绘制连接线
    connected_line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(point_list),
        lines=o3d.utility.Vector2iVector(line_list)
        )
    
    # 设置颜色
    connected_line_set.colors = o3d.utility.Vector3dVector(color_list)
    
    # 合并需要显示的点云
    all_shown = [connected_line_set]
    all_shown.extend(others_shown)

    # 显示图像
    o3d.visualization.draw_geometries(all_shown)

# ******************** 判断两点上方是否有点云, 若有，则返回True, 否则为False 
def above_point(point_1, point_2):

    # 提取两点坐标
    point_1_coor = super_voxel_graph.vertex_xyz[point_1]
    point_2_coor = super_voxel_graph.vertex_xyz[point_2]
    
    # 提取XYZ方向上的极值，供后续矩形框使用
    if point_1_coor[0] <= point_2_coor[0]:
        x_rec_min = point_1_coor[0]
        x_rec_max = point_2_coor[0]
    else:
        x_rec_max = point_1_coor[0]
        x_rec_min = point_2_coor[0]

    if point_1_coor[1] <= point_2_coor[1]:
        y_rec_min = point_1_coor[1]
        y_rec_max = point_2_coor[1]    
    else:
        y_rec_max = point_1_coor[1]
        y_rec_min = point_2_coor[1]
    # <反向了 delete
    if point_1_coor[2] <= point_2_coor[2]:
        z_rec_min = point_1_coor[2]
    else:
        z_rec_min = point_2_coor[2]
    
    x_rec_max = x_rec_max + 5
    x_rec_min = x_rec_min - 5
    y_rec_max = y_rec_max + 5
    y_rec_min = y_rec_min - 5
    # 反向了 delete
    # 矩形框上移10mm
    z_rec_min = z_rec_min - 5

    # 提取矩形区域内的点云
    rectangle_list = xyz_list[
                                np.where(
                                        (xyz_list[:,0] > x_rec_min) & (xyz_list[:,0] < x_rec_max) &
                                        (xyz_list[:,1] > y_rec_min) & (xyz_list[:,1] < y_rec_max) &
                                        (xyz_list[:,2] < z_rec_min)
                                        )
                             ]

    if show_above_point_flag:
        # z反向了 delete
        # 生成矩形的角点
        rectangle_corner_list = np.asarray([
                                            [x_rec_min,y_rec_min,z_rec_min],
                                            [x_rec_min,y_rec_max,z_rec_min],
                                            [x_rec_max,y_rec_max,z_rec_min],
                                            [x_rec_max,y_rec_min,z_rec_min],
                                            [x_rec_min,y_rec_min,z_rec_min-30],
                                            [x_rec_min,y_rec_max,z_rec_min-30],
                                            [x_rec_max,y_rec_max,z_rec_min-30],
                                            [x_rec_max,y_rec_min,z_rec_min-30]
                                            ]) 

        # 生成矩形框点云连线与颜色
        rectangle_Lines = np.array([
            [0,1],[1,2],[2,3],[3,0],
            [4,5],[5,6],[6,7],[7,4],
            [0,4],[1,5],[2,6],[3,7]])
        rectangle_colors = np.array([
            [1,0,0],[1,0,0],[1,0,0],[1,0,0],
            [1,0,0],[1,0,0],[1,0,0],[1,0,0],
            [1,0,0],[1,0,0],[1,0,0],[1,0,0]])
        # 同时显示超体素中心点与完整的点云
        something_shown = [center_point, total_point_cloud]
        line_show(rectangle_Lines, rectangle_corner_list, rectangle_colors, something_shown)

    # 如果选中区域内的点云数量超过阈值100，则认为被遮挡
    if len(rectangle_list) > 100:

        # 同时显示超体素中心点与完整的点云
        # something_shown = [center_point, total_point_cloud]
        
        # 显示连线与点云
        # line_show(rectangle_Lines, rectangle_corner_list, rectangle_colors, something_shown)
        return True
    else:
        return False

# ******************** 比例 ： 异面直线的距离 / 直线上两点的距离
def distance_two_line(p12,v1,v2,dist_2p):

    # 异面直线的距离
    dist = np.dot(np.cross(v1,v2), p12) / np.linalg.norm(np.cross(v1,v2))
    # 比例
    dist_rate = dist / dist_2p

    return dist_rate

# ******************** 初始化参数
def init_parameter():
    global VisionFlag
    global TotalShowFlag
    global axis_flag
    global axis_final_flag
    global file_name
    global resolution
    global k_neighbors
    global distance_threshold
    global angle_threshold
    global show_connect_flag
    global show_k3_flag
    global show_pick_edge_flag
    global show_grip_pose_flag
    global show_above_point_flag
    global show_path_flag
    global show_neighbor_flag
    global show_supervoxel_flag
    global show_conneted_flag
    global show_grip_flag

    # 是否显示点云
    VisionFlag = True
    # 是否显示完整点云
    TotalShowFlag = True
    # 是否显示寻找到的轴线
    axis_flag = False
    # 是否显示最后的轴线
    axis_final_flag = False
    # 是否显示轴线的连接
    show_connect_flag = False
    # 是否显示k3图
    show_k3_flag = False
    # 是否显示选择的抓取段
    show_pick_edge_flag = False
    # 是否显示选择的抓取姿态
    show_grip_pose_flag = False
    # 是否显示矩形框上方点云
    show_above_point_flag = False
    # 是否展示路线
    show_path_flag = False
    # 是否显示临近关系
    show_neighbor_flag = False
    # 是否显示相邻超体素连接线
    show_supervoxel_flag = False
    # 是否显示最终的被连接的所有轴线
    show_conneted_flag = True
    # 是否显示最后的抓取点
    show_grip_flag = False

    # 文件路径
    file_name = \
        'D:/project/tube_pick/super_voxel_workspace/super_voxel/test_data/test/test_string/case3.ply'


    # 分辨率，k临近值
    resolution = 35
    k_neighbors = 70

    # 两轴线连接时的距离阈值与向量角度阈值
    distance_threshold = 70
    angle_threshold = 0.9

# ******************** 寻找三角回路
def find_triangle(vertex_degree_more_or_equal_3):
    # 寻找三角回路
    triangle_list = []
    for nb_0 in vertex_degree_more_or_equal_3:
        # 第一个点的连接的其余节点
        neighbors_for_loop_1 = super_voxel_graph.vertex[nb_0]
        for nb_1 in neighbors_for_loop_1:
            # 第二个点连接的其余节点
            neighbors_for_loop_2 = super_voxel_graph.vertex[nb_1]
            for nb_2 in neighbors_for_loop_2:
                # 第三个点连接的其余节点
                neighbors_for_loop_3 = super_voxel_graph.vertex[nb_2]
                #判断第一点是否在第三点的连接点内
                if nb_0 in neighbors_for_loop_3:
                    loop = [nb_0, nb_1, nb_2]
                    loop.sort()
                    if loop not in triangle_list:
                        triangle_list.append(loop)
    return triangle_list

# ******************** 拿到点云的中心点坐标与正交基
def getPointCloudCenterOrientation(pcCrop):
    eigenvalues, eigenvectors,MeanX,MeanY,MeanZ = PCA(pcCrop)   #PCA方法寻找主轴，得到正交基
    # a,b,c,x,y,z = line_fit_3d_cv(pcCrop)                        #空间直线拟合，得到点云中心坐标
    vx = eigenvectors[:,0]
    vy = eigenvectors[:,1]
    vz = np.cross(vx,vy)
    vector = [vx,vy,vz]
    return MeanX,MeanY,MeanZ,vector

# ******************** 计算PCA的函数 输入：点云，NX3的矩阵，correlation：区分np的cov和corrcoef，sort: 特征值排序 输出：特征值，特征向量
def PCA(data, correlation=False, sort=True):
    dataNumpy = copy.deepcopy(data)
    dataNumpy = np.array(dataNumpy)
    dataLength =  len(data)

    dataNumpyX = []
    dataNumpyY = []
    dataNumpyZ = []

    for i in range(dataLength):
        dataNumpyX.append(dataNumpy[i][0])
        dataNumpyY.append(dataNumpy[i][1])
        dataNumpyZ.append(dataNumpy[i][2])

    MeanX = np.mean(dataNumpyX)
    MeanY = np.mean(dataNumpyY)
    MeanZ = np.mean(dataNumpyZ)

    average_data = np.mean(data,axis=0)       #求 NX3 向量的均值
    decentration_matrix = data - average_data   #去中心化
    H = np.dot(decentration_matrix.T,decentration_matrix)  #求解协方差矩阵 H
    eigenvectors,eigenvalues,eigenvectors_T = np.linalg.svd(H)    # SVD求解特征值、特征向量
    # 屏蔽结束

    if sort:
        sort = eigenvalues.argsort()[::-1]      #降序排列
        eigenvalues = eigenvalues[sort]         #索引
        eigenvectors = eigenvectors[:, sort]

    return eigenvalues, eigenvectors,MeanX,MeanY,MeanZ

# ******************** 直线拟合3D空间点 输入：空间点 输出：空间直线的方向向量与线段中心
def line_fit_3d_cv(pc):
    try:
        a,b,c, x0,y0,z0 = cv2.fitLine(points=pc,
                                    distType=cv2.DIST_L2,
                                    param=0,
                                    reps=0.01,
                                    aeps=0.01)
        return a[0],b[0],c[0],x0[0],y0[0],z0[0]
    except:
        return 0,0,0,0,0,0

# ******************** 为重叠的软管排序
def occlusion_sort(axis_list, axis_connected_list):
    # 储存遮挡信息
    occlusion_list = [[] for i in range(len(axis_list))]

    # 将所有轴线的节点写入列表
    axis_vertex_list = []
    for i in range(len(axis_list)):
        for j in range(len(axis_list[i])):
            axis_vertex_list.append(
                [super_voxel_graph.vertex_xyz[axis_list[i][j]][0],
                super_voxel_graph.vertex_xyz[axis_list[i][j]][1],
                super_voxel_graph.vertex_xyz[axis_list[i][j]][2],
                i,
                j]
                )

    axis_vertex_list = np.array(axis_vertex_list)

    # 遍历所有与其他轴线连接的轴线
    for i in range(len(axis_connected_list)):
        # 连接的次数
        occlusion_times = int(len(axis_connected_list[i])/2)
        # 连接次数大于0
        if occlusion_times > 0:
            # 遍历连接
            for j in range(occlusion_times):
                # 选择的轴线参与连接的两点，与其坐标
                p0 = axis_connected_list[i][j]
                p1 = axis_connected_list[i][j+1]
                p0_xyz = super_voxel_graph.vertex_xyz[p0]
                p1_xyz = super_voxel_graph.vertex_xyz[p1]
                # 一次函数方程 y = kx + b
                k1 = (p1_xyz[1] - p0_xyz[1])/(p1_xyz[0] - p0_xyz[0])
                b1 = p0_xyz[1] - k1 * p0_xyz[0]
                
                # 求两点上空的包围盒， 截取在包围盒中的其他节点
                box_x_min = min(p0_xyz[0], p1_xyz[0]) - 30
                box_x_max = max(p0_xyz[0], p1_xyz[0]) + 30
                box_y_min = min(p0_xyz[1], p1_xyz[1]) - 30
                box_y_max = max(p0_xyz[1], p1_xyz[1]) + 30
                box_z_max = max(p0_xyz[2], p1_xyz[2]) + 5

                # 截取在包围盒中的其他节点
                potential_vertex_above = \
                    axis_vertex_list[
                            np.where(
                                ( axis_vertex_list[:,0] > box_x_min ) & ( axis_vertex_list[:,0] < box_x_max ) &
                                ( axis_vertex_list[:,1] > box_y_min ) & ( axis_vertex_list[:,1] < box_y_max ) &
                                ( axis_vertex_list[:,2] < box_z_max )
                            )
                    ]

                # 节点数量
                ( potential_len, _) = potential_vertex_above.shape
                # 遍历节点
                for k in range(potential_len-1):
                    # 选择其中两个节点
                    p3 = potential_vertex_above[k]
                    p4 = potential_vertex_above[k+1]

                    # numpy转list
                    p3_list = p3[0:3].tolist()
                    p4_list = p4[0:3].tolist()

                    # 如果节点重合，则放弃
                    if p3_list == p0_xyz or p3_list == p1_xyz or p4_list == p0_xyz or p4_list == p1_xyz :
                        continue
                    
                    # 两节点在同一轴线上 & 两节点相邻（索引之差为1）
                    if ( p3[3] == p4[3] ) & ( p4[4] - p3[4] == 1 ):
                        # 一次函数方程 y = kx + b
                        k2 = (p4[1] - p3[1])/(p4[0] - p3[0])
                        b2 = p3[1] - k2 * p3[0]
                        
                        # point_list = [p0_xyz, p1_xyz, p3_list, p4_list]
                        # line_list = [[0,1],[2,3]]
                        # color_list = [[1,0,0],[0,0,1]]
                        # others_shown = [center_point]
                        # line_show(line_list, point_list, color_list, others_shown)

                        # 交叉点
                        x_cross = (b2 - b1) / (k1 - k2)
                        y_cross = k1*x_cross + b1

                        # print('*******************************************')
                        # print('# x_cross : %.4f \t y_cross : %.4f'%(x_cross,y_cross))
                        # print('# x_min : %.4f \t x_max : %.4f'%(min(p0_xyz[0], p1_xyz[0]),max(p0_xyz[0], p1_xyz[0])))
                        # print('# y_min : %.4f \t y_max : %.4f'%(min(p0_xyz[1], p1_xyz[1]),max(p0_xyz[1], p1_xyz[1])))

                        # 如果交叉点在p3p4上，则视为被遮挡
                        if ( x_cross > min(p3[0], p4[0]) ) & ( x_cross < max(p3[0], p4[0]) ) & \
                            ( y_cross > min(p3[1], p4[1]) ) & ( y_cross < max(p3[1], p4[1]) ) :
                            # 遮挡其他管的管的编号，遮挡其他管的两个点的编号*2，被遮挡的管的两个点的编号*2，
                            occlusion_list[i].append(
                                [int(p3[3]),
                                axis_list[int(p3[3])][int(p3[4])],
                                axis_list[int(p4[3])][int(p4[4])], 
                                p0, 
                                p1])
        else:
            pass

    return occlusion_list

# ******************** 挑选抓取点与抓取路径
def choose_pick_point(chosen_axis_index, sorted_axis_list):
    occlusion_case = sorted_axis_list[chosen_axis_index]
    if occlusion_case == []:
        # 提取最高得分轴线的所有节点
        chosen_axis = new_axis_list[chosen_axis_index]
        # 计算该轴线的长度
        chosen_axis_length = calculate(chosen_axis)
        # 得到该轴线一半长度处的两相邻节点
        [pick_0, pick_1] = calculate_half(chosen_axis,chosen_axis_length)
        return pick_0, pick_1, [0,0]
    else:
        # 如果被选中的管只被一根其他管压住
        if len(occlusion_case) == 1:
            
            # 被遮挡的管的相关参数
            occlusion_axis_para = occlusion_case[0]
            # 遮挡被遮挡管的管的编号
            occlusion_axis_index = occlusion_axis_para[0]

            # 遮挡被挑选管的管被其他管压住的情况
            occ_occ_axis_para = sorted_axis_list[occlusion_axis_index]

            if occ_occ_axis_para == []:
                chosen_axis = new_axis_list[occlusion_axis_index]
                # 计算该轴线的长度
                chosen_axis_length = calculate(chosen_axis)
                # 得到该轴线一半长度处的两相邻节点
                [pick_0, pick_1] = calculate_half(chosen_axis,chosen_axis_length)
                return pick_0, pick_1, [0,0]
            else:
                # 提取最高得分轴线的所有节点
                chosen_axis = new_axis_list[chosen_axis_index]
                # 遮挡被挑选的管的管的所有节点
                occ_occ_axis_vertex = new_axis_list[occlusion_axis_index]

                # 遮挡被挑选的管的管被其他管遮挡处的两个节点
                occ_occ_p00_index = occ_occ_axis_para[0][3]
                occ_occ_p01_index = occ_occ_axis_para[0][4]

                # 两个节点对应的坐标
                occ_occ_p00_xyz = np.array(super_voxel_graph.vertex_xyz[occ_occ_p00_index])
                occ_occ_p01_xyz = np.array(super_voxel_graph.vertex_xyz[occ_occ_p01_index])

                # 坐标平均值
                p0_xyz = ( occ_occ_p00_xyz + occ_occ_p01_xyz ) / 2

                # 遮挡被挑选的管的管被其他管遮挡处的两个节点在管轴线中的位置索引
                occ_occ_p00_index_in_axis = occ_occ_axis_vertex.index(occ_occ_p00_index)
                occ_occ_p01_index_in_axis = occ_occ_axis_vertex.index(occ_occ_p01_index)

                # 最大索引与最小索引
                occ_occ_p0_index_in_axis_min = min(occ_occ_p00_index_in_axis, occ_occ_p01_index_in_axis)
                occ_occ_p0_index_in_axis_max = max(occ_occ_p00_index_in_axis, occ_occ_p01_index_in_axis)

                # 被遮挡的管的被遮挡的两个节点
                mid_0_index = occlusion_axis_para[3]
                mid_1_index = occlusion_axis_para[4]
                
                # 两节点的坐标
                mid_0_xyz = np.array(super_voxel_graph.vertex_xyz[mid_0_index])
                mid_1_xyz = np.array(super_voxel_graph.vertex_xyz[mid_1_index])

                # 坐标平均值
                p2_occ_xyz = ( mid_0_xyz + mid_0_xyz ) / 2
                
                # 被遮挡轴线的头和尾
                chosen_axis_start = np.array(super_voxel_graph.vertex_xyz[chosen_axis[0]])
                chosen_axis_end = np.array(super_voxel_graph.vertex_xyz[chosen_axis[-1]])

                start_length = np.linalg.norm( chosen_axis_start - p2_occ_xyz )
                end_length = np.linalg.norm( chosen_axis_end - p2_occ_xyz )

                if start_length < end_length:
                    p2_xyz = chosen_axis_start
                else:
                    p2_xyz = chosen_axis_end

                # 被遮挡的管被以下两个节点遮住
                occ_p10_index = occlusion_axis_para[1]
                occ_p11_index = occlusion_axis_para[2]

                # 两节点的坐标
                occ_p10_xyz = np.array(super_voxel_graph.vertex_xyz[occ_p10_index])
                occ_p11_xyz = np.array(super_voxel_graph.vertex_xyz[occ_p11_index])

                # 坐标平均值
                occ_p1_xyz = ( occ_p10_xyz + occ_p11_xyz ) / 2

                # 遮挡被挑选的管的两个节点在管轴线中的位置索引
                occ_p10_index_in_axis = occ_occ_axis_vertex.index(occ_p10_index)
                occ_p11_index_in_axis = occ_occ_axis_vertex.index(occ_p11_index)
                
                # 最大索引与最小索引
                occ_p1_index_in_axis_min = min(occ_p10_index_in_axis, occ_p11_index_in_axis)
                occ_p1_index_in_axis_max = max(occ_p10_index_in_axis, occ_p11_index_in_axis)
                
                # 判断p0、p1顺序
                # 如果p0索引在后，则p1修正为轴线的第一个节点
                if occ_occ_p0_index_in_axis_min > occ_p1_index_in_axis_max:
                    # p1的编号与坐标
                    p1 = occ_occ_axis_vertex[0]
                    p1_xyz = np.array( super_voxel_graph.vertex_xyz[p1] )

                    # 向量0→1，向量0→2
                    v01 = p1_xyz - p0_xyz
                    v02 = p2_xyz - p0_xyz

                    # v01 v02夹角的cos值
                    theta_cos = np.dot(v01, v02) / np.linalg.norm(v01) / np.linalg.norm(v02)
                    # 夹角的值 + 20°裕量
                    theta = np.arccos(theta_cos) + 20 / 180 * np.pi
                    # 挠度系数 y = =k * x^2
                    coef_deform = 0.00001
                    # 提升动作的系数
                    coef_z = 1
                    # 平移动作的系数
                    coef_translate = 1
                    # 各个节点的移动花费
                    cost_list = []
                    # 节点编号存储
                    vertex_index_list = []
                    # delta_z存储
                    cost_z_list = []
                    # 遍历p0到p1
                    for px in range(1,occ_p1_index_in_axis_min):
                        # px写入列表
                        vertex_index_list.append(px)
                        # px坐标
                        px_xyz = np.array( super_voxel_graph.vertex_xyz[occ_occ_axis_vertex[px]] )
                        # px-p1长度
                        len_px1 = np.linalg.norm(px_xyz - p1_xyz)
                        # p0-px长度
                        len_p0x = np.linalg.norm(p0_xyz - px_xyz)
                        # 某个节点的提升花费
                        cost_z = p2_xyz[2] - px_xyz[2] + coef_deform * len_px1 * len_px1
                        cost_z_list.append(cost_z)
                        # 某个节点的平移花费
                        cost_translate = 2*len_p0x*np.sin(theta/2)
                        # 某个节点的总花费
                        cost_sum = coef_z * cost_z + coef_translate * cost_translate
                        # 花费写入列表
                        cost_list.append(cost_sum)
                    # 选择的节点编号
                    chosed_point_index = cost_list.index(min(cost_list))
                    chosen_delta_z = cost_z_list[chosed_point_index]
                    index_in_axis = vertex_index_list[chosed_point_index]
                    # 节点是否在轴线边缘，若为否，则取节点两侧的点作为抓取参考点
                    if index_in_axis > 0 and index_in_axis < len(occ_occ_axis_vertex)-1:
                        chosed_point_0 = occ_occ_axis_vertex[index_in_axis-1]
                        chosed_point_1 = occ_occ_axis_vertex[index_in_axis+1]
                    if index_in_axis == 0:
                        chosed_point_0 = occ_occ_axis_vertex[index_in_axis]
                        chosed_point_1 = occ_occ_axis_vertex[index_in_axis+1]
                    if index_in_axis == len(occ_occ_axis_vertex) - 1:
                        chosed_point_0 = occ_occ_axis_vertex[index_in_axis-1]
                        chosed_point_1 = occ_occ_axis_vertex[index_in_axis]
                    
                    # 抓取点坐标
                    chosen_point = np.array( super_voxel_graph.vertex_xyz[occ_occ_axis_vertex[index_in_axis]] )
                    # 向量0→chosen
                    v0x = chosen_point - p0_xyz
                    # 求叉乘，如果叉乘小于零，就取角度的负数
                    v_cross_value = np.cross(v0x[0:2], v02[0:2])
                    if v_cross_value < 0:
                        theta = -theta
                    else:
                        pass
                    
                    # 旋转矩阵
                    Rotate_M = [[np.cos(theta),-np.sin(theta)],[np.sin(theta), np.cos(theta)]]
                    Rotate_M = np.array(Rotate_M)
                    # 旋转向量
                    v_rotated = np.dot(Rotate_M, v0x[0:2])

                    # print('# ROTATED #')
                    # print('%.4f %.4f'%(v_rotated[0], v_rotated[1]))

                # 如果p0索引在前，则p1修正为轴线的最后一个节点
                # 类似上一个if
                if occ_occ_p0_index_in_axis_max < occ_p1_index_in_axis_min:
                    p1 = occ_occ_axis_vertex[-1]
                    p1_xyz = np.array( super_voxel_graph.vertex_xyz[p1] )

                    v01 = p1_xyz - p0_xyz
                    v02 = p2_xyz - p0_xyz

                    theta_cos = np.dot(v01, v02) / np.linalg.norm(v01) / np.linalg.norm(v02)
                    theta = np.arccos(theta_cos) + 20 / 180 * np.pi
                    coef_deform = 0.00001
                    coef_z = 1
                    coef_translate = 1
                    cost_list = []
                    vertex_index_list = []
                    # delta_z存储
                    cost_z_list = []
                    for px in range(occ_p1_index_in_axis_max, len(occ_occ_axis_vertex)):
                        vertex_index_list.append(px)
                        px_xyz = np.array( super_voxel_graph.vertex_xyz[occ_occ_axis_vertex[px]] )
                        len_px1 = np.linalg.norm(px_xyz - p1_xyz)
                        len_p0x = np.linalg.norm(p0_xyz - px_xyz)
                        cost_z = p2_xyz[2] - px_xyz[2] + coef_deform * len_px1 * len_px1
                        cost_z_list.append(cost_z)
                        cost_translate = 2*len_p0x*np.sin(theta/2)
                        cost_sum = coef_z * cost_z + coef_translate * cost_translate
                        cost_list.append(cost_sum)
                    chosed_point_index = cost_list.index(min(cost_list))
                    chosen_delta_z = cost_z_list[chosed_point_index]
                    index_in_axis = vertex_index_list[chosed_point_index]
                    if index_in_axis > 0 and index_in_axis < len(occ_occ_axis_vertex)-1:
                        chosed_point_0 = occ_occ_axis_vertex[index_in_axis-1]
                        chosed_point_1 = occ_occ_axis_vertex[index_in_axis+1]
                    if index_in_axis == 0:
                        chosed_point_0 = occ_occ_axis_vertex[index_in_axis]
                        chosed_point_1 = occ_occ_axis_vertex[index_in_axis+1]
                    if index_in_axis == len(occ_occ_axis_vertex) - 1:
                        chosed_point_0 = occ_occ_axis_vertex[index_in_axis-1]
                        chosed_point_1 = occ_occ_axis_vertex[index_in_axis]
                    
                    chosen_point = np.array( super_voxel_graph.vertex_xyz[occ_occ_axis_vertex[index_in_axis]] )
                    v0x = chosen_point - p0_xyz
                    v_cross_value = np.cross(v0x[0:2], v02[0:2])
                    if v_cross_value < 0:
                        theta = -theta
                    else:
                        pass
                    
                    Rotate_M = [[np.cos(theta),-np.sin(theta)],[np.sin(theta), np.cos(theta)]]
                    Rotate_M = np.array(Rotate_M)
                    v_rotated = np.dot(Rotate_M, v0x[0:2])

                    # print('# ROTATED #')
                    # print('%.4f %.4f'%(v_rotated[0], v_rotated[1]))

            # 旋转修正后的路径点
            v_rotated_3d = np.array([v_rotated[0], v_rotated[1], 0]) + p0_xyz
            
            if show_path_flag:
                # 展示路径点的连线
                rotated_list = [p0_xyz.tolist(),chosen_point.tolist(),p2_xyz.tolist(),v_rotated_3d.tolist()]
                rotated_line = [[0,1],[0,2],[0,3]]
                color_list = [[1,0,0],[0,1,0],[0,0,1]]
                others_shown = [center_point]
                line_show(rotated_line,rotated_list,color_list,others_shown)

            return chosed_point_0,chosed_point_1,[1, v_rotated_3d, chosen_delta_z, theta]

def GetTransformationMat(position):
    [x,y,z,rx,ry,rz] = position

    rzSin = np.sin(rz*np.pi/180)
    rzCos = np.cos(rz*np.pi/180)
    rySin = np.sin(ry*np.pi/180)
    ryCos = np.cos(ry*np.pi/180)
    rxSin = np.sin(rx*np.pi/180)
    rxCos = np.cos(rx*np.pi/180)

    rzMat = np.array([
        [rzCos ,-rzSin, 0],
        [rzSin , rzCos, 0],
        [0 ,0 , 1]])

    ryMat = np.array([
        [ryCos ,0, rySin],
        [0 , 1, 0],
        [-rySin ,0 , ryCos]])

    rxMat = np.array([
        [1 , 0, 0],
        [0 , rxCos, -rxSin],
        [0 , rxSin , rxCos]])

    ryxMat = np.dot(ryMat,rxMat)
    rzyxMat = np.dot(rzMat,ryxMat)
    xyzArray = np.array([[x],[y],[z]])
    MatFromGripToBase = np.r_[np.c_[rzyxMat,xyzArray],np.array([[0,0,0,1]])]
    return MatFromGripToBase ,rzyxMat

#功能：判断输入矩阵是否为一旋转矩阵
#输入：待判断矩阵
#输出：
def isRotationMatrix(R) :           
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

#功能：将输入的旋转矩阵化为欧拉角
#输入：3×3矩阵
#输出：欧拉角
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x*180/np.pi, y*180/np.pi, z*180/np.pi])

# ******************** 主程序
#if __name__ == '__main__':
def grip_path():
    global super_voxel_graph
    global n_super_voxels
    global xyz_list
    global new_axis_list
    global center_point
    global total_point_cloud
    MatFromCamToGrip = np.array([
    [ 7.01004560e-01  ,7.13152410e-01 ,-2.49938318e-03 ,-8.62411344e+01],
    [-7.12574723e-01  ,7.00286540e-01 ,-4.28488780e-02 ,-3.19489801e+01],
    [-2.88074962e-02  ,3.18182562e-02  ,9.99078439e-01  ,2.03918376e+01],
    [ 0.00000000e+00 , 0.00000000e+00  ,0.00000000e+00  ,1.00000000e+00]])

    # MatFromCamToGrip = np.array([
    # [  -0.705998    ,0.706867  ,-0.0436553     ,-30.066],
    # [  -0.708173   ,-0.705276  , 0.0328048     ,79.9438],
    # [-0.00760044   ,0.0540756  ,  0.998508     ,13.4887],
    # [          0   ,        0  ,         0     ,      1]])

    startPoint = [410,-320,525,180.00,0.00,-135]
    MatFromGripToBase ,rzyxMat = GetTransformationMat(startPoint)

# ******************** 初始化参数
    init_parameter()

    scene = o3d.io.read_point_cloud( file_name )
    mypc = np.asarray(scene.points)

    # mypc = get_point_cloud.get_pc(1)
    
# ******************** 得到超体素分割后的结果
    t0 = time.time()
    # super_voxel_result = super_voxel.get_label(file_name,resolution,k_neighbors)
    super_voxel_result = super_voxel.get_label(mypc,resolution,k_neighbors)
    t1 = time.time()
    delta_t = t1 - t0
    print('Duration is %.6f'%delta_t)
# ******************** 提取点云数据与超体素中心
    # 得到超体素数量
    n_super_voxels = super_voxel_result[0]

    # 生成超体素图
    super_voxel_graph = graph(n_super_voxels)

    # 储存超体素中心与颜色
    super_voxel_centers_list = np.zeros((n_super_voxels,3))
    super_voxel_color_list = np.zeros((n_super_voxels,3))

    # 得到点云数量，点云标签，xyz坐标
    n_points = super_voxel_result[1]
    label_list = np.array(super_voxel_result[2:n_points+2])
    x_list = np.array(super_voxel_result[n_points+2:2*n_points+2])
    y_list = np.array(super_voxel_result[2*n_points+2:3*n_points+2])
    z_list = np.array(super_voxel_result[3*n_points+2:4*n_points+2])
    xyz_list = np.c_[x_list, np.c_[y_list,np.c_[z_list,label_list]]]

    # 储存点云颜色
    point_cloud_color_list = np.zeros((n_points,3))
    point_cloud_color_grey_list = np.ones((n_points,3))*0.5
    # 备选颜色列表
    color_list = [
        [0,0,1],[0,1,0],[0,1,1],
        [1,0,0],[1,0,1],[1,1,0],
        [0,0.5,1],[0,1,0.5],[0.5,0,1],
        [1,0,0.5],[0.5,1,0],[1,0.5,0],
        [1,1,0.5],[1,0.5,1],[0.5,1,1],
        [0.5,1,0.5],[0.5,0.5,1],[1,0.5,0.5]]

    # 按标签提取点云数据
    for label_number in range(n_super_voxels):

        # 提取标签对应的点云编号
        label_number_list = np.where(label_list == label_number)

        # 提取标签长度，即点云中点的个数，存入图类中
        (label_len,) = label_number_list[0].shape
        super_voxel_graph.vertex_length.append(label_len)

        # 提取点云编号对应的xyz坐标
        x_label_list = x_list[label_number_list[0]]
        y_label_list = y_list[label_number_list[0]]
        z_label_list = z_list[label_number_list[0]]

        # 计算超体素点云的中心坐标
        x_mean = np.mean(x_label_list)
        y_mean = np.mean(y_label_list)
        z_mean = np.mean(z_label_list)

        # 储存超体素点云的中心坐标
        super_voxel_centers_list[label_number,0] = x_mean
        super_voxel_centers_list[label_number,1] = y_mean
        super_voxel_centers_list[label_number,2] = z_mean

        # 将超体素点云中心坐标储存入图类中
        super_voxel_graph.vertex_xyz[label_number] = [x_mean,y_mean,z_mean]

        # 将点云坐标按照标签储存
        super_voxel_graph.super_voxel_label_points_list[label_number] = [ x_label_list, y_label_list,z_label_list ]

        # 给标签对应的点云染色
        color_number = label_number%18
        point_cloud_color_list[label_number_list[0],:] = color_list[color_number]

# ****************** 拼接点云xyz坐标
    total_point_list = np.c_[x_list,np.c_[y_list,z_list]]

    # 创建点云，赋予坐标与颜色值
    total_point_cloud = o3d.geometry.PointCloud()
    total_point_cloud.points = o3d.utility.Vector3dVector(total_point_list)
    total_point_cloud.colors = o3d.utility.Vector3dVector(point_cloud_color_list)

    # 超体素中心
    center_point = o3d.geometry.PointCloud()

    # 创建点云，赋予坐标与颜色值，并显示
    center_point.points = o3d.utility.Vector3dVector(super_voxel_centers_list)
    center_point.colors = o3d.utility.Vector3dVector(super_voxel_color_list)

# ******************** 寻找超体素的临近关系

    # 泰勒展开系数
    coef = 1.0/np.sqrt(3)

    # 寻找超体素的临近关系
    for base_index in range(n_super_voxels):

        # 超体素点云数量下限
        minimum_point_number = 20

        # 如果超体素包含的点云太小则舍弃
        if super_voxel_graph.vertex_length[base_index] < minimum_point_number:
            continue

        # 基准超体素的点云
        base_super_voxel = np.transpose( 
            np.array(
                super_voxel_graph.super_voxel_label_points_list[base_index] 
                    ) 
                                        )
        
        # 从边界向外向内扩张的距离
        interval = 2

        #提取区域在XYZ方向上的上下界
        x_min = min(base_super_voxel[:,0])-interval
        y_min = min(base_super_voxel[:,1])-interval
        z_min = min(base_super_voxel[:,2])-interval

        x_max = max(base_super_voxel[:,0])+interval
        y_max = max(base_super_voxel[:,1])+interval
        z_max = max(base_super_voxel[:,2])+interval

        pick_box_corner_list = np.asarray([
                                            [x_min+2*interval,y_min+2*interval,z_max],
                                            [x_max-2*interval,y_min+2*interval,z_max],
                                            [x_min+2*interval,y_max-2*interval,z_max],
                                            [x_max-2*interval,y_max-2*interval,z_max],
                                            [x_min,y_min,z_max],
                                            [x_max,y_min,z_max],
                                            [x_min,y_max,z_max],
                                            [x_max,y_max,z_max],
                                            ]) 
        # # 生成矩形框点云连线与颜色
        # box_Lines = np.array([[0,1],[1,3],[3,2],[2,0],[4,5],[5,7],[7,6],[6,4]])
        # box_colors = np.array([[1,0,0],[1,0,0],[1,0,0],[1,0,0],[1,0,0],[1,0,0],[1,0,0],[1,0,0]])

        # something_shown = [total_point_cloud]
        # line_show(box_Lines, pick_box_corner_list, box_colors, something_shown)

        # 提取区域内的点云
        neighbors_list = xyz_list[
                                    np.where(
                                            (xyz_list[:,0] > x_min) & (xyz_list[:,0] < x_max) &
                                            (xyz_list[:,1] > y_min) & (xyz_list[:,1] < y_max) &
                                            (xyz_list[:,2] > z_min) & (xyz_list[:,2] < z_max) &
                                            (xyz_list[:,3] != base_index)
                                            )
                                ]

        # 去重，得到潜在相邻超体素编号
        neighbors_label_list = np.unique( neighbors_list[:,3] )
        # 潜在相邻超体素的数量
        neighbors_number = len(neighbors_label_list)
        
        # 如果没有相邻超体素，则跳出
        if neighbors_number == 0:
            # super_voxel_graph.vertex[base_index].append(base_index)
            continue
        
        # 仅保留提取区域内的点云
        base_super_voxel = base_super_voxel[
            np.where(
                    ((base_super_voxel[:,0] > x_min) & (base_super_voxel[:,0] < x_min + 2*interval)) |
                    ((base_super_voxel[:,1] > y_min) & (base_super_voxel[:,1] < y_min + 2*interval)) |
                    ((base_super_voxel[:,2] > z_min) & (base_super_voxel[:,2] < z_min + 2*interval)) |
                    ((base_super_voxel[:,0] < x_max) & (base_super_voxel[:,0] > x_max - 2*interval)) |
                    ((base_super_voxel[:,1] < y_max) & (base_super_voxel[:,1] > y_max - 2*interval)) |
                    ((base_super_voxel[:,2] < z_max) & (base_super_voxel[:,2] > z_max - 2*interval))
                    )
                                            ]

        # 提取区域中点云的数量
        ( point_number, _) = base_super_voxel.shape

        # 该超体素中各点与其余超体素中的点的最小距离
        super_voxel_min_distance_list = 100 * np.ones((neighbors_number,1))

        # 其余超体素标签
        other_super_voxel_index = np.zeros((neighbors_number,1))

        # 其余超体素计数
        count = 0

        # 遍历其余超体素
        for index in neighbors_label_list:

            # int化
            index_int = int(index)

            # 如果基准超体素与相邻超体素的编号组合出现过，则跳过
            if super_voxel_graph.vertex_length[index_int] < minimum_point_number:
                # 计数器+1
                count = count + 1
                continue
            if index_int in super_voxel_graph.vertex[base_index]:
                # 计数器+1
                count = count + 1
                continue
            if base_index in super_voxel_graph.vertex[index_int]:
                # 计数器+1
                count = count + 1
                continue

            # 其余备选超体素的点云
            optional_super_voxel = neighbors_list[np.where(neighbors_list[:,3] == index)][:,0:3]

            # 储存基准超体素中所有点，与其余超体素中的所有点的距离的最小值
            min_distance_list = np.zeros((point_number,1))

            # 选择基准超体素
            for point_index_in_base in range(point_number):

                # 基准超体素中某一点的坐标
                base_point = base_super_voxel[point_index_in_base, :]

                # 存储其余某一超体素中所有点与基准点的向量
                distance_xyz_list = optional_super_voxel - base_point
                distance_list = []
                # 存储其余某一超体素中所有点与基准点的距离，用泰勒展开多项式代替欧氏距离，提高速度
                # for item_v in distance_xyz_list:
                #     distance_list.append(np.linalg.norm( item_v ))
                distance_list = coef * (
                    abs(distance_xyz_list[:,0]) + 
                    abs(distance_xyz_list[:,1]) + 
                    abs(distance_xyz_list[:,2]))

                # 储存最小距离
                min_distance_list[point_index_in_base] = min(distance_list)

            # 储存最小距离
            super_voxel_min_distance_list[count] = min(min_distance_list)

            # 计数器+1
            count = count + 1

        # 遍历潜在相邻超像素
        for k in range(neighbors_number):
            # 若两点间距小于neighbors_min_distance，则认为是相邻点
            neighbors_min_distance = 5
            if super_voxel_min_distance_list[k] < neighbors_min_distance:
                # 将相邻的超像素编号[p1,p2],[p2,p1]写入基准超像素点邻接点列表中
                neighbors_voxel = int(neighbors_label_list[k])
                super_voxel_graph.vertex[base_index].append(neighbors_voxel)
                super_voxel_graph.vertex[neighbors_voxel].append(base_index)

                # 将超像素中心向量p1p2和p2p1写入边缘中
                super_voxel_vector = \
                    np.array(super_voxel_graph.vertex_xyz[neighbors_voxel]) - \
                    np.array(super_voxel_graph.vertex_xyz[base_index])

                super_voxel_graph.edge[base_index,neighbors_voxel,:] = super_voxel_vector
                super_voxel_graph.edge[neighbors_voxel,base_index,:] = -1*super_voxel_vector

    voxel_lines = []
    voxel_colors = []

    for i in range(n_super_voxels):
        # 提取连接线数组
        edges_list = super_voxel_graph.vertex[i]
        # 遍历连接点
        edges_number = len(edges_list)
        for j in range(edges_number):
            voxel_lines.append([i,super_voxel_graph.vertex[i][j]])
            voxel_colors.append([0,0,1])

    if show_neighbor_flag:
        something_shown = [center_point]
        line_show(voxel_lines, super_voxel_centers_list, voxel_colors, something_shown)

# ********************* 剪除图中多余的边
    
    # ********************* 两次循环，删除多余三角回路，暴露出轴线端点
    for i in range(2):

        # 清空梯度
        super_voxel_graph.vertex_degree = []
        # 生成每一节点的梯度
        for i in range(n_super_voxels):
            super_voxel_graph.vertex_degree.append(len(super_voxel_graph.vertex[i]))

        # 找到梯度满足条件的节点
        vertex_degree_numpy_array = np.array( super_voxel_graph.vertex_degree )
        vertex_degree_more_or_equal_3 = np.where( vertex_degree_numpy_array >= 3 )[0]

        # 寻找三角回路
        triangle_list = find_triangle(vertex_degree_more_or_equal_3)

        # 需要被删除的边的列表
        deleted_edge_list = []

        # 遍历在列表中的回路
        for one_loop in triangle_list:
            # 提取梯度
            point_degree_list = [
                super_voxel_graph.vertex_degree[one_loop[0]],
                super_voxel_graph.vertex_degree[one_loop[1]],
                super_voxel_graph.vertex_degree[one_loop[2]]
                ]
            
            # 梯度为2的节点
            point_degree_2 = []
            # 梯度大于等于3的节点
            point_degree_3 = []

            # 将节点按梯度存储入相应的列表中
            for i in range(3):
                if point_degree_list[i] == 2:
                    point_degree_2.append(one_loop[i])
                if point_degree_list[i] >= 3:
                    point_degree_3.append(one_loop[i])

            # 如果只有两个梯度为2的节点
            if len(point_degree_2) == 2:
                # 计算两梯度为2的节点分别与梯度为3的节点的距离
                length_p1 = np.linalg.norm( super_voxel_graph.edge[point_degree_2[0], point_degree_3[0],:] )
                length_p2 = np.linalg.norm( super_voxel_graph.edge[point_degree_2[1], point_degree_3[0],:] )
                # 储存需删除的其中距离较短的一条边与两梯度为2的节点的连线
                if length_p1 < length_p2:
                    deleted_edge_list.append([point_degree_2[0], point_degree_2[1]])
                    deleted_edge_list.append([point_degree_2[0], point_degree_3[0]])
                else:
                    deleted_edge_list.append([point_degree_2[1], point_degree_2[0]])
                    deleted_edge_list.append([point_degree_2[1], point_degree_3[0]])

            # 如果只有一个梯度为2的节点
            if len(point_degree_2) == 1:
                # 计算梯度为2的节点与两梯度为3的节点的距离
                length_p1 = np.linalg.norm( super_voxel_graph.edge[point_degree_2[0], point_degree_3[0],:] )
                length_p2 = np.linalg.norm( super_voxel_graph.edge[point_degree_2[0], point_degree_3[1],:] )
                # 储存需删除的其中距离较短的一条边
                if length_p1 < length_p2:
                    deleted_edge_list.append([point_degree_3[0], point_degree_2[0]])
                else:
                    deleted_edge_list.append([point_degree_3[1], point_degree_2[0]])

        # 遍历需删除的边
        for item in deleted_edge_list:
            # 边的两端点
            p1 = item[0]
            p2 = item[1]

            # 删除某一点的对应连接点
            vertex_neighbor_p1 = super_voxel_graph.vertex[p1]
            vertex_neighbor_p1.remove(p2)
            super_voxel_graph.vertex[p1] = vertex_neighbor_p1

            # 删除某一点的对应连接点
            vertex_neighbor_p2 = super_voxel_graph.vertex[p2]
            vertex_neighbor_p2.remove(p1)       
            super_voxel_graph.vertex[p2] = vertex_neighbor_p2

    # ********************* 两次循环 删除多余梯度为1的节点的边，避免干扰轴线端点判断
    for i in range(2):

        # 清空梯度
        super_voxel_graph.vertex_degree = []
        # 生成每一节点的梯度
        for i in range(n_super_voxels):
            super_voxel_graph.vertex_degree.append(len(super_voxel_graph.vertex[i]))

        # 找到梯度满足条件的节点
        vertex_degree_numpy_array = np.array( super_voxel_graph.vertex_degree )
        # 找到梯度为1的节点
        vertex_degree_1 = np.where( vertex_degree_numpy_array == 1 )[0]

        # 需要被删除的边对应的节点
        deleted_vertex_list = []
        # 遍历梯度为1的节点A
        for vd_1 in vertex_degree_1:
            # 该节点A连接的节点B
            vd_1_neighbor = super_voxel_graph.vertex[vd_1][0]
            # 节点B连接的节点列表
            vd_1_n_n = copy.deepcopy(super_voxel_graph.vertex[vd_1_neighbor])
            # 列表长度
            vd_neighbor_list_len = len(vd_1_n_n)
            # 若列表长度为2，则放弃
            if vd_neighbor_list_len == 2:
                continue
            # 若列表长度为3，则计算向量夹角
            if vd_neighbor_list_len >= 3:
                v12_cos_flag = False
                # 去除节点A自身
                vd_1_n_n.remove(vd_1)
                # 遍历其余的节点
                for k in range(vd_neighbor_list_len-1):
                    for m in range(k+1, vd_neighbor_list_len-1):
                        # 向量一
                        v1 = np.array(super_voxel_graph.edge[vd_1_neighbor, vd_1_n_n[k],:])
                        # 向量二
                        v2 = np.array(super_voxel_graph.edge[vd_1_neighbor, vd_1_n_n[m],:])
                        # 向量cos值
                        v12_cos = np.dot(v1, v2) / np.linalg.norm(v1) / np.linalg.norm(v2)
                        # 若向量值小于-0.6，则需要删除节点A的边
                        if v12_cos < -0.6:
                            v12_cos_flag = True
                        else:
                            pass
                # 将节点A的边加入删除列表
                if v12_cos_flag == True:
                    deleted_vertex_list.append([vd_1, vd_1_neighbor])
                else:
                    pass
        
        # 遍历删除列表
        for deleted_vertex_item in deleted_vertex_list:
            # 梯度为1的节点
            vertex_d1_deleted = deleted_vertex_item[0]
            # 连接梯度为1的节点的节点
            vertex_d1_connected = deleted_vertex_item[1]
            # 清空 边
            super_voxel_graph.edge[vertex_d1_deleted, vertex_d1_connected, :] = [0, 0, 0]
            super_voxel_graph.edge[vertex_d1_connected, vertex_d1_deleted, :] = [0, 0, 0]
            # 删除 边
            super_voxel_graph.vertex[vertex_d1_deleted] = []
            super_voxel_graph.vertex[vertex_d1_connected].remove(vertex_d1_deleted)


    # 显示结果
    if show_supervoxel_flag:
        # 相邻超像素的连接线
        voxel_lines = []
        voxel_colors = []
        for i in range(n_super_voxels):
            # 提取连接线数组
            edges_list = super_voxel_graph.vertex[i]
            # 遍历连接点
            edges_number = len(edges_list)
            for j in range(edges_number):
                voxel_lines.append([i,super_voxel_graph.vertex[i][j]])
                voxel_colors.append([0,0,1])
        
        something_shown = [center_point]
        line_show(voxel_lines, super_voxel_centers_list, voxel_colors, something_shown)

    # ********************* 删除K3完全图中的某一个点连接的边

    # 生成每一节点的梯度
    for i in range(n_super_voxels):
        super_voxel_graph.vertex_degree[i] = len(super_voxel_graph.vertex[i])

    # 找到梯度满足条件的节点
    vertex_degree_numpy_array = np.array( super_voxel_graph.vertex_degree )
    vertex_degree_3 = np.where( vertex_degree_numpy_array >= 3 )[0]

    # 寻找三角回路
    triangle_list = find_triangle(vertex_degree_3)
    
    # 储存需要被删除的边对应的点
    deleted_vertex_list = []
    # 寻找K3图，删除中心点
    for item in vertex_degree_3:
        item_degree = super_voxel_graph.vertex_degree[item]
        # 储存连接点
        loop_edge = copy.deepcopy(super_voxel_graph.vertex[item])
        # 中心点连接的边与其他三角回路里的边重叠的次数
        loop_neighbor = 0

        edge_list = []
        # 遍历连接点
        for other_item in loop_edge:
            # 单独一条边的重叠次数
            edge_neighbor = 0
            # 创建中心点连接的边
            loop_one_edge = [ other_item, item ]
            # 遍历所有三角回路
            for tri_loop in triangle_list:
                # 重叠的点
                same_element_list = [ x for x in loop_one_edge if x in tri_loop]
                # 若重叠两个点，则认为边重叠次数+1
                if len(same_element_list) == 2:
                    loop_neighbor = loop_neighbor + 1
                    edge_neighbor = edge_neighbor + 1
            if edge_neighbor >= 2:
                edge_list.append(other_item)

        if len(edge_list) >= 2:
            for c1 in range(len(edge_list)):
                for c2 in range(c1+1,len(edge_list)):
                    p1 = edge_list[c1]
                    p2 = edge_list[c2]
                    if p2 in super_voxel_graph.vertex[p1]:
                         
                        p1_connected = super_voxel_graph.vertex[p1]
                        p2_connected = super_voxel_graph.vertex[p2]

                        p1p2_same = [x for x in p1_connected if x in p2_connected]
                        p1p2_same.remove(item)
                        if len(p1p2_same) == 0:
                            continue
                        p3 = p1p2_same[0]

                        line_k3_list = []
                        line_set_k3 =  [
                                        [item,p1], [item,p2], [item,p3],
                                        [p1,p2], [p1,p3], [p2,p3]
                                        ]
                        
                        for set_index in range(6):
                            line_k3_list.append(
                                                np.linalg.norm(
                                                    super_voxel_graph.edge[
                                                        line_set_k3[set_index][0],
                                                        line_set_k3[set_index][1],
                                                        :]
                                                                )
                                                )

                        min_index = line_k3_list.index(min(line_k3_list))
                        min_line = line_set_k3[min_index][:]
                        deleted_vertex_list.append(min_line)

                        # 显示K3中心点与连接边
                        # 相邻超像素的连接线
                        if show_k3_flag:

                            voxel_lines = []
                            voxel_colors = []
                            
                            voxel_lines.append([min_line[0], min_line[1]])
                            voxel_colors.append([0.3,0.3,0])

                            voxel_lines.append([item, p1])
                            voxel_lines.append([item, p2])
                            voxel_lines.append([p1, p2])
                            voxel_colors.append([0,1,0])
                            voxel_colors.append([0,1,0])
                            voxel_colors.append([0,1,0])

                            for one_vertex in loop_edge:
                                voxel_lines.append([item, one_vertex])
                                voxel_colors.append([1,0,0])

                            for i in range(n_super_voxels):
                                # 提取连接线数组
                                edges_list = super_voxel_graph.vertex[i]
                                # 遍历连接点
                                edges_number = len(edges_list)
                                for j in range(edges_number):
                                    voxel_lines.append([i,super_voxel_graph.vertex[i][j]])
                                    voxel_colors.append([0,0,1])

                            something_shown = [center_point]
                            line_show(voxel_lines, super_voxel_centers_list, voxel_colors, something_shown)
                        # 显示K3中心点与连接边

    # 遍历待删除的边缘
    for deleted_edge in deleted_vertex_list:
        edge_vertex_1 = deleted_edge[0]
        edge_vertex_2 = deleted_edge[1]

        # 清空 边
        super_voxel_graph.edge[edge_vertex_1, edge_vertex_2, :] = [0, 0, 0]
        super_voxel_graph.edge[edge_vertex_1, edge_vertex_2, :] = [0, 0, 0]
        # 删除 边
        connected_vertex_1 = super_voxel_graph.vertex[edge_vertex_1]
        connected_vertex_2 = super_voxel_graph.vertex[edge_vertex_2]
        if edge_vertex_2 in connected_vertex_1:
            super_voxel_graph.vertex[edge_vertex_1].remove(edge_vertex_2)
        if edge_vertex_1 in connected_vertex_2:
            super_voxel_graph.vertex[edge_vertex_2].remove(edge_vertex_1)

    # ********************* 删除三角回路里的重叠的边缘

    # 生成每一节点的梯度
    for i in range(n_super_voxels):
        super_voxel_graph.vertex_degree[i] = len(super_voxel_graph.vertex[i])

    # 找到梯度满足条件的节点
    vertex_degree_numpy_array = np.array( super_voxel_graph.vertex_degree )
    vertex_degree_more_or_equal_3 = np.where( vertex_degree_numpy_array >= 3 )[0]

    # 寻找三角回路
    triangle_list = find_triangle(vertex_degree_more_or_equal_3)

    # 删除三角回路里的重叠的边缘
    deleted_edge_triangle_list = []
    for i in range(len(triangle_list)):
        for j in range(i+1, len(triangle_list)):
            # 提取边缘
            target_loop = triangle_list[i]
            other_loop = triangle_list[j]
            # 两个回路里的共用节点
            same_element_list = [ x for x in target_loop if x in other_loop]
            # 共用节点如果是两个
            if len(same_element_list) == 2:
                if same_element_list not in deleted_edge_triangle_list:
                    # 存入待删除边缘的列表
                    deleted_edge_triangle_list.append(same_element_list)

    # 遍历待删除的边缘
    for edge_item in deleted_edge_triangle_list:
        v1 = edge_item[0]
        v2 = edge_item[1]
        # 清空 边
        super_voxel_graph.edge[v1, v2, :] = [0, 0, 0]
        super_voxel_graph.edge[v2, v1, :] = [0, 0, 0]
        # 删除 边
        super_voxel_graph.vertex[v1].remove(v2)
        super_voxel_graph.vertex[v2].remove(v1)

    # ********************* 两次循环 删除多余梯度为1的节点的边，避免干扰轴线端点判断
    for i in range(2):

        # 清空梯度
        super_voxel_graph.vertex_degree = []
        # 生成每一节点的梯度
        for i in range(n_super_voxels):
            super_voxel_graph.vertex_degree.append(len(super_voxel_graph.vertex[i]))

        # 找到梯度满足条件的节点
        vertex_degree_numpy_array = np.array( super_voxel_graph.vertex_degree )
        # 找到梯度为1的节点
        vertex_degree_1 = np.where( vertex_degree_numpy_array == 1 )[0]

        # 需要被删除的边对应的节点
        deleted_vertex_list = []
        # 遍历梯度为1的节点A
        for vd_1 in vertex_degree_1:
            # 该节点A连接的节点B
            vd_1_neighbor = super_voxel_graph.vertex[vd_1][0]
            # 节点B连接的节点列表
            vd_1_n_n = copy.deepcopy(super_voxel_graph.vertex[vd_1_neighbor])
            # 列表长度
            vd_neighbor_list_len = len(vd_1_n_n)
            # 若列表长度为2，则放弃
            if vd_neighbor_list_len == 2:
                continue
            # 若列表长度为3，则计算向量夹角
            if vd_neighbor_list_len >= 3:
                v12_cos_flag = False
                # 去除节点A自身
                vd_1_n_n.remove(vd_1)
                # 遍历其余的节点
                for k in range(vd_neighbor_list_len-1):
                    for m in range(k+1, vd_neighbor_list_len-1):
                        # 向量一
                        v1 = np.array(super_voxel_graph.edge[vd_1_neighbor, vd_1_n_n[k],:])
                        # 向量二
                        v2 = np.array(super_voxel_graph.edge[vd_1_neighbor, vd_1_n_n[m],:])
                        # 向量cos值
                        v12_cos = np.dot(v1, v2) / np.linalg.norm(v1) / np.linalg.norm(v2)
                        # 若向量值小于-0.6，则需要删除节点A的边
                        if v12_cos < -0.6:
                            v12_cos_flag = True
                        else:
                            pass
                # 将节点A的边加入删除列表
                if v12_cos_flag == True:
                    deleted_vertex_list.append([vd_1, vd_1_neighbor])
                else:
                    pass
        
        # 遍历删除列表
        for deleted_vertex_item in deleted_vertex_list:
            # 梯度为1的节点
            vertex_d1_deleted = deleted_vertex_item[0]
            # 连接梯度为1的节点的节点
            vertex_d1_connected = deleted_vertex_item[1]
            # 清空 边
            super_voxel_graph.edge[vertex_d1_deleted, vertex_d1_connected, :] = [0, 0, 0]
            super_voxel_graph.edge[vertex_d1_connected, vertex_d1_deleted, :] = [0, 0, 0]
            # 删除 边
            super_voxel_graph.vertex[vertex_d1_deleted] = []
            super_voxel_graph.vertex[vertex_d1_connected].remove(vertex_d1_deleted)

    # 显示结果
    if VisionFlag:
        o3d.visualization.draw_geometries([total_point_cloud])
        # 相邻超像素的连接线
        voxel_lines = []
        voxel_colors = []
        for i in range(n_super_voxels):
            # 提取连接线数组
            edges_list = super_voxel_graph.vertex[i]
            # 遍历连接点
            edges_number = len(edges_list)
            for j in range(edges_number):
                voxel_lines.append([i,super_voxel_graph.vertex[i][j]])
                voxel_colors.append([0,0,1])
        
        # 显示完整点云
        if TotalShowFlag:
            something_shown = [center_point, total_point_cloud]
            line_show(voxel_lines, super_voxel_centers_list, voxel_colors, something_shown)
        else:
            something_shown = [center_point]
            line_show(voxel_lines, super_voxel_centers_list, voxel_colors, something_shown)

    # # 将超体素的连接关系写入txt
    # for i in range(n_super_voxels):
    #     with open('data.txt','a') as f:
    #         f.write(str(i)+'\t'+str(super_voxel_graph.vertex[i])+'\n')    #可以是随便对文件的操作
    #     f.close()

    # 生成每一节点的梯度
    for i in range(n_super_voxels):
        super_voxel_graph.vertex_degree[i] = len(super_voxel_graph.vertex[i])

    # 找到梯度满足条件的节点
    vertex_degree_numpy_array = np.array( super_voxel_graph.vertex_degree )
    vertex_degree_more_or_equal_3 = np.where( vertex_degree_numpy_array >= 3 )[0]
    vertex_degree_1 = np.where( vertex_degree_numpy_array == 1 )[0]

    # 将轴线储存入数组
    axis_list = [[] for i in range(len(vertex_degree_1))]

    axis_number = 0
    # 将梯度为1的节点视为软管断点，找软管轴线
    for now_vertex in vertex_degree_1:

        # 生成节点掩膜,1为可选，0为不可选
        vertex_mask = np.ones((n_super_voxels,1))

        # 添加起始点
        axis_list[axis_number].append(now_vertex)

        # 是否继续运行
        run_flag = True

        # 启动时标志
        start_flag = True

        while run_flag:

            # 关闭当前节点
            vertex_mask[now_vertex] = 0

            # 获得节点的连接点
            next_vertex_list = copy.deepcopy(super_voxel_graph.vertex[now_vertex])

            # 如果是第一步搜索，则另作处理
            if start_flag == True:
                last_vertex = copy.deepcopy(now_vertex)
                now_vertex = copy.deepcopy(next_vertex_list[0])
                axis_list[axis_number].append(now_vertex)
                start_flag = False
                continue

            # 生成从当前节点指向前一节点的向量
            now_last_vertex_vector = super_voxel_graph.edge[now_vertex, last_vertex, :]

            # 向量夹角cos的最小值与最小时的超像素编号
            index_maximum_cos = None
            maximum_cos = -100

            # 遍历连接点
            for next_vertex in next_vertex_list:
                # 如果下一点不可行，则放弃
                if vertex_mask[next_vertex] == 0:
                    continue
                # 生成从下一节点指向当前节点的向量
                next_last_vertex_vector = super_voxel_graph.edge[next_vertex, now_vertex, :]  
                #计算两向量的夹角cos值
                now_next_cos = \
                    np.dot(now_last_vertex_vector,next_last_vertex_vector) \
                    / np.linalg.norm(now_last_vertex_vector) \
                    / np.linalg.norm(next_last_vertex_vector)
                
                # 如果当前cos值大于已知最大cos值，则更新 
                if now_next_cos > maximum_cos:
                    maximum_cos = copy.deepcopy(now_next_cos)
                    index_maximum_cos = next_vertex
                else:
                    pass

            # 如果cos值超过了允许的最小值，则视为可行，否则则结束寻路
            valid_minimum_cos = 0.8
            if maximum_cos > valid_minimum_cos:
                # 更新节点
                last_vertex = copy.deepcopy(now_vertex)
                now_vertex = copy.deepcopy(index_maximum_cos)
                axis_list[axis_number].append(now_vertex)
            else:
                run_flag = False
        
        # 相邻超像素的连接线
        axis_lines = []
        axis_colors = []
        for i in range(len(axis_list[axis_number])-1):
            # 提取连接线数组
            axis_lines.append([ axis_list[axis_number][i], axis_list[axis_number][i+1] ])
            axis_colors.append([0,0,1])
        
        if axis_flag:
            something_shown = [center_point]
            line_show(axis_lines, super_voxel_centers_list, axis_colors, something_shown)

        axis_number = axis_number + 1

    # 删除重叠的轴线中，较短的一根
    deleted_list = []
    for i in range(len(vertex_degree_1)):
        for j in range(i+1,len(vertex_degree_1)):
            # 取两根轴线
            first_list = axis_list[i]
            second_list = axis_list[j]
            # 两轴线的重叠部分
            same_element_list = [x for x in first_list if x in second_list]
            # 如果两轴线完全重合，则删除一根
            if (len(same_element_list) == len(first_list) 
                & len(same_element_list) == len(second_list)):
                deleted_list.append(j)
                continue
            # 如果两轴线的重合节点数大于一，则保留更长的那一根
            if len(same_element_list) > 1:
                if (len(first_list) - len(second_list)) >= 0:
                    deleted_list.append(j)
                else:
                    deleted_list.append(i)

    # 生成剪除后的轴线列表
    axis_list_final = []
    for i in range(len(vertex_degree_1)):
        if i not in deleted_list:
            axis_list_final.append(axis_list[i])

# ********************* 寻找轴线
    axis_number = len(axis_list_final)

    axis_candidate = axis(axis_number)

    # 储存每根轴线的节点编号、端点坐标、末端向量等数据
    for axis_candidate_index in range(axis_number):
        # 储存每根轴线的节点编号
        axis_candidate_vertex = axis_list_final[axis_candidate_index]
        axis_candidate.vertex.append(axis_candidate_vertex)
        axis_candidate_vertex_number = len(axis_candidate_vertex)

        # 每根轴线的端点坐标
        p00 = np.array(super_voxel_graph.vertex_xyz[axis_candidate_vertex[0]])
        p01 = np.array(super_voxel_graph.vertex_xyz[axis_candidate_vertex[1]])
        p10 = np.array(super_voxel_graph.vertex_xyz[axis_candidate_vertex[axis_candidate_vertex_number-1]])
        p11 = np.array(super_voxel_graph.vertex_xyz[axis_candidate_vertex[axis_candidate_vertex_number-2]])

        # 每根轴线的末端向量与归一化
        axis_candidate_vector_1 = p01 - p00
        axis_candidate_vector_2 = p11 - p10
        axis_candidate_vector_1 = axis_candidate_vector_1 / np.linalg.norm(axis_candidate_vector_1)
        axis_candidate_vector_2 = axis_candidate_vector_2 / np.linalg.norm(axis_candidate_vector_2)

        # 写入类
        axis_candidate.vector.append([axis_candidate_vector_1,axis_candidate_vector_2])
        axis_candidate.terminal.append([p00,p10])
        axis_candidate.terminal_index.append([axis_candidate_vertex[0],axis_candidate_vertex[axis_candidate_vertex_number-1]])

    # 储存连接的轴线的列表
    connected_axis = []

    # 计算哪两根轴线可以连接起来
    # 轴线1
    for axis_candidate_index_0 in range(axis_number):

        # 轴线1端点坐标
        terminal_00 = axis_candidate.terminal[axis_candidate_index_0][0]
        terminal_01 = axis_candidate.terminal[axis_candidate_index_0][1]

        # 轴线1末端向量
        vector_00 = axis_candidate.vector[axis_candidate_index_0][0]
        vector_01 = axis_candidate.vector[axis_candidate_index_0][1]

        # 轴线1末端编号
        index_00 = axis_candidate.terminal_index[axis_candidate_index_0][0]
        index_01 = axis_candidate.terminal_index[axis_candidate_index_0][1]

         # 轴线2
        for axis_candidate_index_1 in range(axis_candidate_index_0+1, axis_number):

            # 轴线2端点坐标
            terminal_10 = axis_candidate.terminal[axis_candidate_index_1][0]
            terminal_11 = axis_candidate.terminal[axis_candidate_index_1][1]

            # 轴线2末端向量
            vector_10 = axis_candidate.vector[axis_candidate_index_1][0]
            vector_11 = axis_candidate.vector[axis_candidate_index_1][1]

            # 轴线2末端编号
            index_10 = axis_candidate.terminal_index[axis_candidate_index_1][0]
            index_11 = axis_candidate.terminal_index[axis_candidate_index_1][1]

            # 两轴线端点的连接线
            vector_2t_00 = terminal_00 - terminal_10
            vector_2t_01 = terminal_00 - terminal_11
            vector_2t_10 = terminal_01 - terminal_10
            vector_2t_11 = terminal_01 - terminal_11

            # 计算连接线距离
            distance_00 = np.linalg.norm(vector_2t_00)
            distance_01 = np.linalg.norm(vector_2t_01)
            distance_10 = np.linalg.norm(vector_2t_10)
            distance_11 = np.linalg.norm(vector_2t_11)

            # 如果连接线小于距离阈值（以下四块代码均遵循本注释）
            if distance_00 < distance_threshold:
                # 如果连接线为0，则为端点重合的两条线，将其加入列表中
                if distance_00 == 0:
                    # connected_axis.append([axis_candidate_index_0,axis_candidate_index_1,0,0])
                    continue
                # 计算角度
                vector_angle = np.dot(vector_00,vector_10)
                vector_angle_1 = np.dot(vector_00,vector_2t_00 / distance_00)
                vector_angle_2 = np.dot(vector_10,vector_2t_00 / distance_00)
                #如果角度cos值大于阈值
                if (-vector_angle > angle_threshold) and \
                    (vector_angle_1 > angle_threshold) and \
                    (-vector_angle_2 > angle_threshold):
                    # 末端点的异面空间直线距离与两末端点距离只比，若比例过大，则放弃连接
                    rate = distance_two_line(vector_2t_00, vector_00, vector_10, distance_00)
                    if rate > 0.2:
                        continue
                    # 两末端点上空存在一定数量的点云，则认为这两段轴线被遮挡了，实则为同一条点云
                    if above_point(index_00,index_10) == True:
                        # 展示连接起来的轴线
                        if show_connect_flag:
                            axis_lines_copy = copy.deepcopy(voxel_lines)
                            axis_colors_copy = copy.deepcopy(voxel_colors)
                            axis_lines_copy.append([index_00, index_10])
                            axis_colors_copy.append([1,0,0])
                            something_shown = [center_point]
                            line_show(axis_lines_copy, super_voxel_centers_list, axis_colors_copy, something_shown)
                        # 储存被连接的轴线，0,0表示第一根轴线的头，连接第二根轴线的头
                        connected_axis.append([axis_candidate_index_0,axis_candidate_index_1,0,0])

            if distance_01 < distance_threshold:
                if distance_01 == 0:
                    # connected_axis.append([axis_candidate_index_0,axis_candidate_index_1,0,1])
                    continue
                vector_angle = np.dot(vector_00,vector_11)
                vector_angle_1 = np.dot(vector_00,vector_2t_01 / distance_01)
                vector_angle_2 = np.dot(vector_11,vector_2t_01 / distance_01)
                if (-vector_angle > angle_threshold) and \
                    (vector_angle_1 > angle_threshold) and \
                    (-vector_angle_2 > angle_threshold):
                    rate = distance_two_line(vector_2t_01, vector_00, vector_11, distance_01)
                    if rate > 0.2:
                        continue
                    if above_point(index_00,index_11) == True:
                        if show_connect_flag:
                            axis_lines_copy = copy.deepcopy(voxel_lines)
                            axis_colors_copy = copy.deepcopy(voxel_colors)
                            axis_lines_copy.append([index_00, index_11])
                            axis_colors_copy.append([1,0,0])
                            something_shown = [center_point]
                            line_show(axis_lines_copy, super_voxel_centers_list, axis_colors_copy, something_shown)
                        connected_axis.append([axis_candidate_index_0,axis_candidate_index_1,0,1])

            if distance_10 < distance_threshold:
                if distance_10 == 0:
                    # connected_axis.append([axis_candidate_index_0,axis_candidate_index_1,1,0])
                    continue
                vector_angle = np.dot(vector_01,vector_10)
                vector_angle_1 = np.dot(vector_01,vector_2t_10 / distance_10)
                vector_angle_2 = np.dot(vector_10,vector_2t_10 / distance_10)
                if (-vector_angle > angle_threshold) and \
                    (vector_angle_1 > angle_threshold) and \
                    (-vector_angle_2 > angle_threshold):
                    rate = distance_two_line(vector_2t_10, vector_01, vector_10, distance_10)
                    if rate > 0.2:
                        continue
                    if above_point(index_01,index_10) == True:
                        if show_connect_flag:
                            axis_lines_copy = copy.deepcopy(voxel_lines)
                            axis_colors_copy = copy.deepcopy(voxel_colors)
                            axis_lines_copy.append([index_01, index_10])
                            axis_colors_copy.append([1,0,0])
                            something_shown = [center_point]
                            line_show(axis_lines_copy, super_voxel_centers_list, axis_colors_copy, something_shown)
                        connected_axis.append([axis_candidate_index_0,axis_candidate_index_1,1,0])

            if distance_11 < distance_threshold:
                if distance_11 == 0:
                    # connected_axis.append([axis_candidate_index_0,axis_candidate_index_1,1,1])
                    continue
                vector_angle = np.dot(vector_01,vector_11)
                vector_angle_1 = np.dot(vector_01,vector_2t_11 / distance_11)
                vector_angle_2 = np.dot(vector_11,vector_2t_11 / distance_11)
                if (-vector_angle > angle_threshold) and \
                    (vector_angle_1 > angle_threshold) and \
                    (-vector_angle_2 > angle_threshold):
                    rate = distance_two_line(vector_2t_11, vector_01, vector_11, distance_11)
                    if rate > 0.2:
                        continue
                    if above_point(index_01,index_11) == True:
                        if show_connect_flag:
                            axis_lines_copy = copy.deepcopy(voxel_lines)
                            axis_colors_copy = copy.deepcopy(voxel_colors)
                            axis_lines_copy.append([index_01, index_11])
                            axis_colors_copy.append([1,0,0])
                            something_shown = [center_point]
                            line_show(axis_lines_copy, super_voxel_centers_list, axis_colors_copy, something_shown)
                        connected_axis.append([axis_candidate_index_0,axis_candidate_index_1,1,1])

    # 储存轴线连接点
    axis_connected_list = []
    # 储存被连接的轴线
    new_axis_list = []
    # 储存需要被连接的轴线的编号
    connected_axis_number = []
    # 遍历需要被连接的轴线
    for connected_item in connected_axis:
        # 取出轴线编号
        axis_0 = connected_item[0]
        axis_1 = connected_item[1]
        # 取出轴线节点列表
        axis_0_vertex = copy.deepcopy(axis_candidate.vertex[axis_0])
        axis_1_vertex = copy.deepcopy(axis_candidate.vertex[axis_1])
        # 储存需要被连接的轴线的编号
        connected_axis_number.append(axis_0)
        connected_axis_number.append(axis_1)

        # 拼接轴线
        if (connected_item[2] == 0) & (connected_item[3] == 0):
            # 倒转第一根轴线，并拼接第二根轴线
            new_axis = copy.deepcopy(axis_0_vertex[::-1])
            new_axis.extend(axis_1_vertex)

            # 将连接起的那段轴线的向量写入edge属性中
            p0 = np.array(super_voxel_graph.vertex_xyz[axis_0_vertex[0]])
            p1 = np.array(super_voxel_graph.vertex_xyz[axis_1_vertex[0]])

            super_voxel_graph.edge[axis_0_vertex[0],axis_1_vertex[0],:] = p1 - p0
            super_voxel_graph.edge[axis_1_vertex[0],axis_0_vertex[0],:] = p0 - p1

            # 储存连接点
            axis_connected_list.append([axis_0_vertex[0],axis_1_vertex[0]])

        if (connected_item[2] == 0) & (connected_item[3]) == 1:
            # 依次拼接第二、第一根轴线
            new_axis = copy.deepcopy(axis_1_vertex)
            new_axis.extend(axis_0_vertex)

            p0 = np.array(super_voxel_graph.vertex_xyz[axis_1_vertex[-1]])
            p1 = np.array(super_voxel_graph.vertex_xyz[axis_0_vertex[0]])

            super_voxel_graph.edge[axis_1_vertex[-1],axis_0_vertex[0],:] = p1 - p0
            super_voxel_graph.edge[axis_0_vertex[0],axis_1_vertex[-1],:] = p0 - p1

            # 储存连接点
            axis_connected_list.append([axis_1_vertex[-1],axis_0_vertex[0]])

        if (connected_item[2] == 1) & (connected_item[3] == 0):
            # 依次拼接第一、第二根轴线
            new_axis = copy.deepcopy(axis_0_vertex)
            new_axis.extend(axis_1_vertex)

            p0 = np.array(super_voxel_graph.vertex_xyz[axis_0_vertex[-1]])
            p1 = np.array(super_voxel_graph.vertex_xyz[axis_1_vertex[0]])

            super_voxel_graph.edge[axis_0_vertex[-1],axis_1_vertex[0],:] = p1 - p0
            super_voxel_graph.edge[axis_1_vertex[0],axis_0_vertex[-1],:] = p0 - p1

            # 储存连接点
            axis_connected_list.append([axis_0_vertex[-1],axis_1_vertex[0]])

        if (connected_item[2] == 1) & (connected_item[3] == 1):
            # 依次拼接第一、倒转的第二根轴线
            new_axis = copy.deepcopy(axis_0_vertex)
            new_axis.extend(axis_1_vertex[::-1])

            p0 = np.array(super_voxel_graph.vertex_xyz[axis_0_vertex[-1]])
            p1 = np.array(super_voxel_graph.vertex_xyz[axis_1_vertex[-1]])

            super_voxel_graph.edge[axis_0_vertex[-1],axis_1_vertex[-1],:] = p1 - p0
            super_voxel_graph.edge[axis_1_vertex[-1],axis_0_vertex[-1],:] = p0 - p1

            # 储存连接点
            axis_connected_list.append([axis_0_vertex[-1],axis_1_vertex[-1]])

        # 储存拼接后的轴线
        new_axis_list.append(new_axis)

    # 所有轴线的编号
    previous_axis_list = [x for x in range(axis_number)]
    # 所有没有被拼接的轴线的编号
    other_axis = [x for x in previous_axis_list if x not in connected_axis_number]

    # 将没有被拼接的轴线添加进列表
    for i in other_axis:
        new_axis_list.append(axis_list_final[i])
        axis_connected_list.append([])
    
    # 计算轴线间的堆叠关系
    sorted_axis_list = occlusion_sort(new_axis_list, axis_connected_list)

    # 显示连接
    connected_axis_list = []
    # 显示连接轴线
    connected_axis_color_list = []
    # 生成显示用的连接轴线列表
    for axis_item in new_axis_list:
        for i in range(len(axis_item)-1):
            connected_axis_list.append([axis_item[i],axis_item[i+1]])
            connected_axis_color_list.append([0,0,1])
    if show_conneted_flag:
        # 显示连接后的轴线
        something_shown = [center_point]
        line_show(connected_axis_list, super_voxel_centers_list, connected_axis_color_list, something_shown)

# ********************* 挑选某一个轴线，计算抓取姿态
    # 将连接后的轴线送入connect_axis类
    axis_pick =  connect_axis(new_axis_list, axis_connected_list)
    # 计算每条轴线的得分
    axis_score = axis_pick.score(sorted_axis_list)
    # 选择其中最高得分的轴线
    chosen_axis_index = axis_score.index(max(axis_score))
    # 计算抓取点与路径点
    pick_0, pick_1, move_path = choose_pick_point(chosen_axis_index, sorted_axis_list)

    # # 提取最高得分轴线的所有节点
    # chosen_axis = new_axis_list[chosen_axis_index]
    # # 计算该轴线的长度
    # chosen_axis_length = calculate(chosen_axis)
    # # 得到该轴线一半长度处的两相邻节点
    # [pick_0, pick_1] = calculate_half(chosen_axis,chosen_axis_length)

    # ****************** 获取两点上方点云，计算PCA，得到抓取姿态
    pick_point_0 = np.array(super_voxel_graph.vertex_xyz[pick_0])
    pick_point_1 = np.array(super_voxel_graph.vertex_xyz[pick_1])

    pick_point_vector_01 = pick_point_0 - pick_point_1
    # 沿节点连线方向拓展
    pick_box_long_axis_p0 = 0.25 * pick_point_vector_01 + pick_point_0
    pick_box_long_axis_p1 = -0.25 * pick_point_vector_01 + pick_point_1

    pick_point_vector_01_normal = pick_point_vector_01[0:2] / np.linalg.norm(pick_point_vector_01[0:2])
    pick_point_vector_orthog = np.array([pick_point_vector_01_normal[1], pick_point_vector_01_normal[0]])
    
    # 沿节点连线的垂线方向拓展
    pick_box_corner = np.zeros((4,2))
    pick_box_corner[0,:] = pick_box_long_axis_p0[0:2] + 12*pick_point_vector_orthog
    pick_box_corner[1,:] = pick_box_long_axis_p0[0:2] - 12*pick_point_vector_orthog
    pick_box_corner[2,:] = pick_box_long_axis_p1[0:2] + 12*pick_point_vector_orthog
    pick_box_corner[3,:] = pick_box_long_axis_p1[0:2] - 12*pick_point_vector_orthog

    #包围盒
    pick_box_max_x = np.max(pick_box_corner[:,0])
    pick_box_min_x = np.min(pick_box_corner[:,0])
    pick_box_max_y = np.max(pick_box_corner[:,1])
    pick_box_min_y = np.min(pick_box_corner[:,1])

    # 反向 delete
    pick_box_max_z = max(pick_box_long_axis_p0[2], pick_box_long_axis_p1[2])

    # 提取矩形区域内的点云
    pick_box_list = xyz_list[
                                np.where(
                                        (xyz_list[:,0] > pick_box_min_x) & (xyz_list[:,0] < pick_box_max_x) &
                                        (xyz_list[:,1] > pick_box_min_y) & (xyz_list[:,1] < pick_box_max_y) &
                                        # 反向 delete
                                        (xyz_list[:,2] < pick_box_max_z)
                                        )
                             ]

    #获得立方体中的点云
    x,y,z,Vector = getPointCloudCenterOrientation(pick_box_list[:,0:3])

    # 求姿态
    vx = Vector[0]
    vy = Vector[1]
    vz = Vector[2]
    vxTrans = vx.T
    vyTrans = vy.T
    vzTrans = vz.T
    if vzTrans[2] > 0:
        vxTrans = -1*vxTrans
        vyTrans = -1*vyTrans
        vzTrans = -1*vzTrans
    VectorTrans = -1*np.array([vxTrans.T,vyTrans.T,vzTrans.T])
    VectorTrans = VectorTrans.T                                 #旋转矩阵

    theMat = np.r_[np.c_[VectorTrans,np.array([[0],[0],[0]])],np.array([[0,0,0,1]])]
    # ****************** 获取两点上方点云，计算PCA，得到抓取姿态

    gripCenter = np.array([x,y,z])
    gripCord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50)
    gripCord.transform(theMat)
    gripCord.translate(gripCenter)

    if show_grip_flag:
        # 如果存在路径点
        if move_path[0] == 1:
            moveCenter = move_path[1]
            moveCord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50)
            r_t = -40/180*np.pi
            rotate_m = np.array([
                [np.cos(r_t), -np.sin(r_t), 0, 0],
                [np.sin(r_t), np.cos(r_t), 0, 0],
                [0,0,1,0],
                [0,0,0,1]
            ])
            moveCord.transform(theMat)
            moveCord.transform(rotate_m)
            moveCord.translate(moveCenter)
            o3d.visualization.draw_geometries([total_point_cloud, gripCord, moveCord])
        else:
            total_point_cloud.colors = o3d.utility.Vector3dVector(point_cloud_color_grey_list)
            o3d.visualization.draw_geometries([total_point_cloud, gripCord])

    if False:
        # 生成矩形的角点
        pick_box_corner_list = np.asarray([
                                            [pick_box_min_x,pick_box_min_y,pick_box_z],
                                            [pick_box_min_x,pick_box_max_y,pick_box_z],
                                            [pick_box_max_x,pick_box_max_y,pick_box_z],
                                            [pick_box_max_x,pick_box_min_y,pick_box_z]

                                            ]) 
        # 生成矩形框点云连线与颜色
        box_Lines = np.array([[0,1],[1,2],[2,3],[3,0]])
        box_colors = np.array([[1,0,0],[0,1,0],[1,1,0],[1,0,1]])

        something_shown = [center_point, total_point_cloud]
        line_show(box_Lines, pick_box_corner_list, box_colors, something_shown)

    # 展示挑选出的轴线与选取的轴线抓取段
    if show_pick_edge_flag:
        chosen_axis_list = []
        chosen_axis_color_list = []

        chosen_axis_list.append([pick_0, pick_1])
        chosen_axis_color_list.append([0,0,1])

        for i in range(len(chosen_axis)-1):
            chosen_axis_list.append([chosen_axis[i], chosen_axis[i+1]])
            chosen_axis_color_list.append([1,0,0])
        something_shown = [center_point]
        line_show(chosen_axis_list, super_voxel_centers_list, chosen_axis_color_list, something_shown)

    gripCenterInBase = np.dot(MatFromCamToGrip,np.array([gripCenter[0],gripCenter[1],gripCenter[2],1]))
    gripCenterInBase = np.dot(MatFromGripToBase,gripCenterInBase)
    gripCenterInBase = gripCenterInBase + np.array([0,0,90,0])
    EulerAng = rotationMatrixToEulerAngles(np.dot(rzyxMat,VectorTrans))  #欧拉角
    EulerAng = EulerAng + np.array([0,0,90])

    if move_path[0] == 0:
        point_1st = gripCenterInBase + np.array([0,0,90,0])
        point_2nd = copy.deepcopy(gripCenterInBase)
        point_3rd = gripCenterInBase + np.array([0,0,200,0])
        point_4th = np.array([85.997,-267.884,300,0])

        gripCenterInBase_list = [point_1st,point_2nd, point_3rd, point_4th]
        EulerAng_list = [EulerAng, EulerAng, EulerAng, np.array([180,0,135.0])]

    if move_path[0] == 1:
        end_point = move_path[1]
        delta_z = move_path[2]
        theta_angle = move_path[3] * 180 / np.pi
        point_1st = gripCenterInBase + np.array([0,0,90,0])
        point_2nd = copy.deepcopy(gripCenterInBase)
        point_3rd = gripCenterInBase + np.array([0,0,90,0])
        point_4th_in_grip = np.dot(MatFromCamToGrip, np.array([end_point[0],end_point[1],end_point[2],1]))
        point_4th = np.dot(MatFromGripToBase, point_4th_in_grip) + np.array([0,0,90,0]) + np.array([0,0,90,0])

        gripCenterInBase_list = [point_1st,point_2nd, point_3rd, point_4th]
        EulerAng_list = [EulerAng, EulerAng, EulerAng, np.array([180, 0, EulerAng[2]-theta_angle])]

    return gripCenterInBase_list, EulerAng_list