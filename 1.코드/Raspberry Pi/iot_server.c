/* 서울기술교육센터 AIoT */
/* author : KSH*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <mysql/mysql.h>
#include <time.h>

#define BUF_SIZE 100
#define MAX_CLNT 32
#define ID_SIZE 10
#define ARR_CNT 5

typedef struct {
    int fd;
    char* from;
    char* to;
    char* msg;
    int len;
} MSG_INFO;

typedef struct {
    int index;
    int fd;
    char ip[20];
    char id[ID_SIZE];
    char pw[ID_SIZE];
    int pillTypeCount;
} CLIENT_INFO;

void* clnt_connection(void* arg);
void send_msg(MSG_INFO* msg_info, CLIENT_INFO* first_client_info);
void error_handling(char* msg);
void log_file(char* msgstr);
void send_pill_info_to_client(CLIENT_INFO* client, MYSQL* conn);
void* morning_sender(void* arg);

int clnt_cnt = 0;
pthread_mutex_t mutx;
CLIENT_INFO client_info[MAX_CLNT];
pthread_t morning_thread;

void load_id_pw_from_file() {
    FILE* fp = fopen("idpasswd.txt", "r");
    if (!fp) error_handling("idpasswd.txt open error");
    for (int i = 0; i < MAX_CLNT; i++) {
        client_info[i].index = i;
        client_info[i].fd = -1;
        strcpy(client_info[i].ip, "");
        client_info[i].pillTypeCount = 0;
        if (fscanf(fp, "%s %s", client_info[i].id, client_info[i].pw) != 2) break;
    }
    fclose(fp);
}

int main(int argc, char* argv[])
{
    int serv_sock, clnt_sock;
    struct sockaddr_in serv_adr, clnt_adr;
    int clnt_adr_sz;
    int sock_option = 1;
    pthread_t t_id[MAX_CLNT] = { 0 };
    int str_len = 0;
    int i;
    char idpasswd[(ID_SIZE * 2) + 3];
    char* pToken;
    char* pArray[ARR_CNT] = { 0 };
    char msg[BUF_SIZE];

    load_id_pw_from_file();

    if (argc != 2) {
        printf("Usage : %s <port>\n", argv[0]);
        exit(1);
    }
    fputs("IoT Server Start!!\n", stdout);

    if (pthread_mutex_init(&mutx, NULL))
        error_handling("mutex init error");

    serv_sock = socket(PF_INET, SOCK_STREAM, 0);

    memset(&serv_adr, 0, sizeof(serv_adr));
    serv_adr.sin_family = AF_INET;
    serv_adr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_adr.sin_port = htons(atoi(argv[1]));

    setsockopt(serv_sock, SOL_SOCKET, SO_REUSEADDR, (void*)&sock_option, sizeof(sock_option));
    if (bind(serv_sock, (struct sockaddr*)&serv_adr, sizeof(serv_adr)) == -1)
        error_handling("bind() error");

    if (listen(serv_sock, 5) == -1)
        error_handling("listen() error");

    pthread_create(&morning_thread, NULL, morning_sender, NULL);
    pthread_detach(morning_thread);

    while (1) {
        clnt_adr_sz = sizeof(clnt_adr);
        clnt_sock = accept(serv_sock, (struct sockaddr*)&clnt_adr, &clnt_adr_sz);
        if (clnt_cnt >= MAX_CLNT) {
            printf("socket full\n");
            shutdown(clnt_sock, SHUT_WR);
            continue;
        }
        else if (clnt_sock < 0) {
            perror("accept()");
            continue;
        }

        str_len = read(clnt_sock, idpasswd, sizeof(idpasswd));
        idpasswd[str_len] = '\0';

        if (str_len > 0) {
            i = 0;
            pToken = strtok(idpasswd, "[:]");

            while (pToken != NULL) {
                pArray[i] = pToken;
                if (i++ >= ARR_CNT)
                    break;
                pToken = strtok(NULL, "[:]");
            }
            for (i = 0; i < MAX_CLNT; i++) {
                if (!strcmp(client_info[i].id, pArray[0])) {
                    if (client_info[i].fd != -1) {
                        sprintf(msg, "[%s] Already logged!\n", pArray[0]);
                        write(clnt_sock, msg, strlen(msg));
                        log_file(msg);
                        shutdown(clnt_sock, SHUT_WR);
                        client_info[i].fd = -1;
                        break;
                    }
                    if (!strcmp(client_info[i].pw, pArray[1])) {
                        strcpy(client_info[i].ip, inet_ntoa(clnt_adr.sin_addr));
                        pthread_mutex_lock(&mutx);
                        client_info[i].index = i;
                        client_info[i].fd = clnt_sock;
                        client_info[i].pillTypeCount = 0;
                        clnt_cnt++;
                        pthread_mutex_unlock(&mutx);
                        sprintf(msg, "[%s] New connected! (ip:%s,fd:%d,sockcnt:%d)\n", pArray[0], inet_ntoa(clnt_adr.sin_addr), clnt_sock, clnt_cnt);
                        log_file(msg);
                        write(clnt_sock, msg, strlen(msg));
                        pthread_create(t_id + i, NULL, clnt_connection, (void*)(client_info + i));
                        pthread_detach(t_id[i]);
                        break;
                    }
                }
            }
            if (i == MAX_CLNT) {
                sprintf(msg, "[%s] Authentication Error!\n", pArray[0]);
                write(clnt_sock, msg, strlen(msg));
                log_file(msg);
                shutdown(clnt_sock, SHUT_WR);
            }
        }
        else
            shutdown(clnt_sock, SHUT_WR);

    }
    return 0;
}

void* clnt_connection(void* arg)
{
    CLIENT_INFO* client_info = (CLIENT_INFO*)arg;
    int str_len = 0;
    int index = client_info->index;
    char msg[BUF_SIZE];
    char to_msg[MAX_CLNT * ID_SIZE + 1];
    int i = 0;
    char* pToken;
    char* pArray[ARR_CNT] = { 0 };
    char strBuff[BUF_SIZE * 2] = { 0 };

    MSG_INFO msg_info;
    CLIENT_INFO* first_client_info;

    MYSQL* conn = NULL;
    conn = mysql_init(NULL);
    if (!(mysql_real_connect(conn, "localhost", "iot", "pwiot", "mini_project", 0, NULL, 0))) {
        fprintf(stderr, "DB ERROR : %s[%d]\n", mysql_error(conn), mysql_errno(conn));
        conn = NULL;
    }

    time_t last_send = time(NULL);

    first_client_info = (CLIENT_INFO*)((void*)client_info - (void*)(sizeof(CLIENT_INFO) * index));

    int setend_flag = 0;

    fd_set rfds;
    struct timeval tv;

    while (1)
    {
        FD_ZERO(&rfds);
        FD_SET(client_info->fd, &rfds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int retval = select(client_info->fd + 1, &rfds, NULL, NULL, &tv);

       

        if (retval > 0 && FD_ISSET(client_info->fd, &rfds)) {
            memset(msg, 0x0, sizeof(msg));
            str_len = read(client_info->fd, msg, sizeof(msg) - 1);
            if (str_len <= 0)
                break;
            msg[str_len] = '\0';

            if (strncmp(msg, "[HKT_LIN]Setend", 15) == 0) {
                printf("Setend received! pillTypeCount = %d, conn=%p\n", client_info->pillTypeCount, conn);
                setend_flag = 1;
                continue;
            }
            if (strncmp(msg, "[HKT_LIN]PILLTYPE@", 17) == 0) {
                char* tok;
                int field = 0, type_count = 0;
                char tmp[BUF_SIZE];
                strcpy(tmp, msg);

                tok = strtok(tmp, "[@]");
                while (tok) {
                    if (field == 2) type_count = atoi(tok);
                    tok = strtok(NULL, "[@]");
                    field++;
                }
                client_info->pillTypeCount = type_count;
                printf("pillTypeCount for %s set to %d\n", client_info->id, client_info->pillTypeCount);

                continue;
            }
            if (strncmp(msg, "[HKT_LIN]PILL", 12) == 0 && strstr(msg, "_DOSE@")) {
                int pill_num = 0, dose = 0;
                sscanf(msg, "[HKT_LIN]PILL%d_DOSE@%d", &pill_num, &dose);

                if (conn && pill_num >= 1 && pill_num <= 5) {
                    char sql_cmd[256], valstr[32];
                    if (pill_num == 1) sprintf(valstr, "%d,0,0,0,0", dose);
                    if (pill_num == 2) sprintf(valstr, "0,%d,0,0,0", dose);
                    if (pill_num == 3) sprintf(valstr, "0,0,%d,0,0", dose);
                    if (pill_num == 4) sprintf(valstr, "0,0,0,%d,0", dose);
                    if (pill_num == 5) sprintf(valstr, "0,0,0,0,%d", dose);

                    sprintf(sql_cmd,
                        "INSERT INTO pills (NAME, Pill1, Pill2, Pill3, Pill4, Pill5) VALUES ('%s', %s) "
                        "ON DUPLICATE KEY UPDATE Pill%d=%d",
                        client_info->id, valstr, pill_num, dose
                    );
                    if (mysql_query(conn, sql_cmd)) {
                        fprintf(stderr, "DB UPDATE ERROR: %s[%d]\n", mysql_error(conn), mysql_errno(conn));
                    }
                    else {
                        printf("Inserted/Updated %s Pill%d to %d\n", client_info->id, pill_num, dose);
                    }
                }
                continue;
            }
            pToken = strtok(msg, "[:]");
            i = 0;
            while (pToken != NULL) {
                pArray[i] = pToken;
                if (i++ >= ARR_CNT)
                    break;
                pToken = strtok(NULL, "[:]");
            }

            msg_info.fd = client_info->fd;
            msg_info.from = client_info->id;
            msg_info.to = pArray[0];
            sprintf(to_msg, "[%s]%s", msg_info.from, pArray[1]);
            msg_info.msg = to_msg;
            msg_info.len = strlen(to_msg);

            sprintf(strBuff, "msg : [%s->%s] %s", msg_info.from, msg_info.to, pArray[1]);
            log_file(strBuff);
            send_msg(&msg_info, first_client_info);
        }
    }
    if (conn) mysql_close(conn);

    close(client_info->fd);

    sprintf(strBuff, "Disconnect ID:%s (ip:%s,fd:%d,sockcnt:%d)\n", client_info->id, client_info->ip, client_info->fd, clnt_cnt - 1);
    log_file(strBuff);

    pthread_mutex_lock(&mutx);
    clnt_cnt--;
    client_info->fd = -1;
    pthread_mutex_unlock(&mutx);

    return 0;
}


void send_msg(MSG_INFO* msg_info, CLIENT_INFO* first_client_info)
{
    int i = 0;

    if (!strcmp(msg_info->to, "ALLMSG")) {
        for (i = 0; i < MAX_CLNT; i++)
            if ((first_client_info + i)->fd != -1)
                write((first_client_info + i)->fd, msg_info->msg, msg_info->len);
    }
    else if (!strcmp(msg_info->to, "IDLIST")) {
        char* idlist = (char*)malloc(ID_SIZE * MAX_CLNT);
        msg_info->msg[strlen(msg_info->msg) - 1] = '\0';
        strcpy(idlist, msg_info->msg);

        for (i = 0; i < MAX_CLNT; i++) {
            if ((first_client_info + i)->fd != -1) {
                strcat(idlist, (first_client_info + i)->id);
                strcat(idlist, " ");
            }
        }
        strcat(idlist, "\n");
        write(msg_info->fd, idlist, strlen(idlist));
        free(idlist);
    }
    else
        for (i = 0; i < MAX_CLNT; i++)
            if ((first_client_info + i)->fd != -1)
                if (!strcmp(msg_info->to, (first_client_info + i)->id))
                    write((first_client_info + i)->fd, msg_info->msg, msg_info->len);
}

void error_handling(char* msg)
{
    fputs(msg, stderr);
    fputc('\n', stderr);
    exit(1);
}

void log_file(char* msgstr)
{
    fputs(msgstr, stdout);
}

void send_pill_info_to_client(CLIENT_INFO* client, MYSQL* conn) {
    if (client->pillTypeCount > 0 && conn) {
        char sql_cmd[128];
        MYSQL_RES* result;
        MYSQL_ROW row;
        sprintf(sql_cmd, "SELECT Pill1, Pill2, Pill3, Pill4, Pill5 FROM pills WHERE NAME='%s'", client->id);
        if (mysql_query(conn, sql_cmd) == 0) {
            result = mysql_store_result(conn);
            if (result && (row = mysql_fetch_row(result))) {
                char outmsg[128];
                int pill[5] = { 0,0,0,0,0 };
                for (int j = 0; j < 5; j++)
                    if (row[j]) pill[j] = atoi(row[j]);
                sprintf(outmsg, "[HKT_STM]start");
                for (int j = 0; j < 5; j++) {
                    char t[8];
                    sprintf(t, "@%d", (j < client->pillTypeCount) ? pill[j] : 0);
                    strcat(outmsg, t);
                }
                strcat(outmsg, "\n");
                write(client->fd, outmsg, strlen(outmsg));
                printf("Send to %s: %s", client->id, outmsg);
            }
            if (result) mysql_free_result(result);
        }
    }
}

void* morning_sender(void* arg) {
    int last_hour = -1; 
    
    while (1) {
        time_t now = time(NULL);
        struct tm* tm_info = localtime(&now);
        
        if (tm_info->tm_hour == 8 && last_hour != 8) {
            MYSQL* conn = mysql_init(NULL);
            if (mysql_real_connect(conn, "localhost", "iot", "pwiot", "mini_project", 0, NULL, 0)) {
                pthread_mutex_lock(&mutx);
                for (int i = 0; i < MAX_CLNT; i++) {
                    if (client_info[i].fd != -1 && client_info[i].pillTypeCount > 0) {
                        send_pill_info_to_client(&client_info[i], conn);
                    }
                }
                pthread_mutex_unlock(&mutx);
                mysql_close(conn);
            }
            last_hour = 8;  
        }
        else if (tm_info->tm_hour != 8) {
            last_hour = -1;
        }
        
        usleep(100000);  
    }
    return NULL;
}
