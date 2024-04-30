#include <iostream>
#include <sys/usbdi.h>
#include <hw/usb_devices.h>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <stdio.h>
#include <string>
#include <stdint.h>
#include <sys/mman.h>

#include <C:/Users/nverdier/ide-7.0-workspace/Camera_USB/src/include/linux/videodev2.h>

using namespace std;
uint8_t *buffer;
void Open_file(std::string chemin) {
    std::ifstream fichier(chemin.c_str());
    if (fichier.is_open()) {
        std::cout << "Le fichier a été ouvert avec succès." << std::endl;
        fichier.close();
    } else {
        std::cout << "Impossible d'ouvrir le fichier." << std::endl;
    }
}

void Write_file(std::string chemin, std::string contenu) {
    std::ofstream fichier;
    fichier.open(chemin.c_str());
    if (fichier.is_open()) {
        fichier << contenu;
        fichier.close();
    } else {
        std::cout << "Impossible d'ouvrir le fichier pour l'écriture." << std::endl;
    }
}
// Fonction pour lire à partir d'un fichier sur la clé USB
std::string Read_file(std::string chemin) {
    std::string contenu;
    std::ifstream fichier(chemin.c_str());
    if (fichier.is_open()) {
        getline(fichier, contenu);
        fichier.close();
    } else {
        std::cout << "Impossible d'ouvrir le fichier pour la lecture." << std::endl;
    }
    return contenu;
}



static int xioctl(int fd, int request, void *arg)
{
        int r;

        do r = ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);

        return r;
}

int print_caps(int fd)
{
        struct v4l2_capability caps = {};
        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps))
        {
                perror("Querying Capabilities");
                return 1;
        }

        printf( "Driver Caps:\n"
                "  Driver: \"%s\"\n"
                "  Card: \"%s\"\n"
                "  Bus: \"%s\"\n"
                "  Version: %d.%d\n"
                "  Capabilities: %08x\n",
                caps.driver,
                caps.card,
                caps.bus_info,
                (caps.version>>16)&&0xff,
                (caps.version>>24)&&0xff,
                caps.capabilities);

        if ((caps.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            fprintf(stderr, "Device is a video capture device\n");
        }

        if ((caps.capabilities & V4L2_CAP_STREAMING)) {
            fprintf(stderr, "Devices does support streaming i/o\n");
        }

        if (!(caps.capabilities & V4L2_CAP_READWRITE)) {
            fprintf(stderr, "Device does not support read i/o\n");
        }

        struct v4l2_cropcap cropcap = {0};
        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl (fd, VIDIOC_CROPCAP, &cropcap))
        {
                perror("Querying Cropping Capabilities");
                return 1;
        }

        printf( "Camera Cropping:\n"
                "  Bounds: %dx%d+%d+%d\n"
                "  Default: %dx%d+%d+%d\n"
                "  Aspect: %d/%d\n",
                cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
                cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
                cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);

        int support_grbg10 = 0;

        struct v4l2_fmtdesc fmtdesc = {0};
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        char fourcc[5] = {0};
        char c, e;
        printf("  FMT : CE Desc\n--------------------\n");
        while (0 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc))
        {
                strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
                if (fmtdesc.pixelformat == V4L2_PIX_FMT_MJPEG)
                    support_grbg10 = 1;
                c = fmtdesc.flags & 1? 'C' : ' ';
                e = fmtdesc.flags & 2? 'E' : ' ';
                printf("  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
                fmtdesc.index++;
        }

        if (!support_grbg10)
        {
            printf("Doesn't support GRBG10.\n");
            return 1;
        }

        struct v4l2_format fmt = {0};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = 752;
        fmt.fmt.pix.height = 480;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
        {
            perror("Setting Pixel Format");
            return 1;
        }

        strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
        printf( "Selected Camera Mode:\n"
                "  Width: %d\n"
                "  Height: %d\n"
                "  PixFmt: %s\n"
                "  Field: %d\n",
                fmt.fmt.pix.width,
                fmt.fmt.pix.height,
                fourcc,
                fmt.fmt.pix.field);
        return 0;
}

int init_mmap(int fd)
{
    struct v4l2_requestbuffers req = {0};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        perror("Requesting Buffer");
        return 1;
    }

    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
    {
        perror("Querying Buffer");
        return 1;
    }

    buffer = static_cast<std::uint8_t*>(mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset));
    printf("Length: %d\nAddress: %p\n", buf.length, buffer);
    printf("Image Length: %d\n", buf.bytesused);

    return 0;
}

int capture_image(int fd)
{
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;

    if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
    {
        perror("Query Buffer");
        return 1;
    }

    if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
    {
        perror("Start Capture");
        return 1;
    }


    close(fd);
    return 0;
}


int main() {
    // Chemin du fichier sur la clé USB
    std::string chemin = "/dev/video0";
    int fd = open(chemin.c_str(), O_RDWR);
    if (fd == -1) {
        std::cout << "Impossible d'ouvrir le fichier." << std::endl;
    }
    std::cout << "Le fichier a été ouvert avec succès." << std::endl;

    // Écrire dans le fichier
    //Read_file(chemin, "test 1,2");
    if(print_caps(fd))
        return 1;

    if(init_mmap(fd))
        return 1;

    if(capture_image(fd))
        return 1;
    // Lire à partir du fichier
    //std::string contenu = Read_file(chemin);
    //std::cout << "Contenu du fichier: " << contenu << std::endl;

    return 0;
}
