#include<stdio.h>
typedef struct {
    short x;
    short y;
    short z;
}Vector3i;

typedef struct {
    Vector3i acc;
    Vector3i gyro;
    short temp;
}dump_half_sec_type;


#define DUMP_LEN_ONE_SEC 8*1024
#define DUMP_LEN (5*60*60*DUMP_LEN_ONE_SEC)
dump_half_sec_type dump_half_sec[DUMP_LEN];
#define GRAVITY_MSS 9.80665f
#define ICM20689_ACCEL_SCALE_1G    (GRAVITY_MSS / 2048.0f) // 16g
#define GYRO_SCALE (0.0174532f / 16.4f) // radis to degree per LSB

main()
{
    FILE * stream;
    int i;
    stream = fopen("./dump","r");
    fread(dump_half_sec,sizeof(dump_half_sec_type),DUMP_LEN,stream);
    fclose(stream);
    FILE *fd = fopen("./dump_acc.txt","w");
    FILE *fd1 = fopen("./dump_gyro.txt","w");
    for(i=0;i<DUMP_LEN;i++)
    {
        
        if((i & 1) == 0)
        {
            fprintf(fd, "%f,%f,%f,%f\n", dump_half_sec[i].acc.x*ICM20689_ACCEL_SCALE_1G,
               dump_half_sec[i].acc.y*ICM20689_ACCEL_SCALE_1G,
               dump_half_sec[i].acc.z*ICM20689_ACCEL_SCALE_1G,
               dump_half_sec[i].temp/340.0f + 36.53f);
        }
        fprintf(fd1, "%f,%f,%f,%f\n", 
               dump_half_sec[i].gyro.x*GYRO_SCALE,
               dump_half_sec[i].gyro.y*GYRO_SCALE,
               dump_half_sec[i].gyro.z*GYRO_SCALE,
               dump_half_sec[i].temp/340.0f + 36.53f);
    }
}
