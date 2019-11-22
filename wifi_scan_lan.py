import socket
import threading

# 创建接收路由列表
routers = []
# 创建互斥锁
lock = threading.Lock()
# 设置需要扫描的端口号列表
port_list = ['1111']

# 定义查询路由函数
def search_routers():
    # 获取本地ip地址列表
    local_ips = socket.gethostbyname_ex(socket.gethostname())[2]
    # print(local_ips)
    # 存放线程列表池
    all_threads = []
    # 循环本地网卡IP列表
    for ip in local_ips:
        for i in range(1, 255):
            # 把网卡IP"."进行分割,生成每一个可用地址的列表
            array = ip.split('.')
            # 获取分割后的第四位数字，生成该网段所有可用IP地址
            array[3] = str(i)
            # 把分割后的每一可用地址列表，用"."连接起来，生成新的ip
            new_ip = '.'.join(array)
            # print(new_ip)
            # 遍历需要扫描的端口号列表
            for port in port_list:
                dst_port = int(port)
                # 循环创建线程去链接该地址
                t = threading.Thread(target=check_ip, args=(new_ip, dst_port) )
                t.start()
                # 把新建的线程放到线程池
                all_threads.append(t)
    # 循环阻塞主线程，等待每一字子线程执行完，程序再退出
    for t in all_threads:
        t.join()

# 创建访问IP列表方法
def check_ip(new_ip, port):
    # 创建TCP套接字，链接新的ip列表
    scan_link = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 设置链接超时时间
    scan_link.settimeout(2)
    # 链接地址(通过指定我们 构造的主机地址，和扫描指定端口)
    result = scan_link.connect_ex((new_ip, port))
    #
    scan_link.close()
    # print(result)
    # 判断链接结果
    if result == 0:
        # 加锁
        lock.acquire()

        print(new_ip, '\t\t端口号%s开放' % port)
        routers.append((new_ip, port))
        # 释放锁
        lock.release()
    # print(routers)

print("正在扫描..., 请稍等...")

# 启动程序入口
if __name__ == '__main__':
    # 启动扫描程序
    search_routers()
