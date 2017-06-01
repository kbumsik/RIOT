#include "winc1500_socket.h"
#include <stdint.h>
#include <stdio.h>

#include "socket/include/socket.h"

extern int state_socket;
void winc1500_socket_cb(SOCKET sock, uint8_t msg_type, void *payload)
{
    switch (msg_type) {
        case SOCKET_MSG_BIND: {	/* Called from bind(). For TCP/UDP */
            tstrSocketBindMsg *bind_msg = (tstrSocketBindMsg*)payload;
            
            if (0 == bind_msg->status) {
                /* Success */
                puts("Socket binding is successful.");
                state_socket = 1;
            } else if (0 > bind_msg->status) {
                /* Failed */
                puts("Socket binding failed.");
            } else {
                /* Unknown */
                puts("Unknown socket binding error.");
            }
        }
            break;

        case SOCKET_MSG_LISTEN: { /* Called from listen(). For TCP */
            tstrSocketListenMsg *listen_msg = (tstrSocketListenMsg*)payload;
            
            if (0 == listen_msg->status) {
                /* Success */
                printf("Listening on socket %d started.\n", sock);
                state_socket = 1;
            } else if (0 > listen_msg->status) {
                /* Failed */
                printf("Listening on socket %d failed.\n", sock);
            } else {
                /* Unknown */
                puts("Unknown socket binding error.");
            }
        }
            break;

        case SOCKET_MSG_DNS_RESOLVE: {
                /* Unidentified behavior */
                /* There is a callback function explicitly for DNS Resolve */
        }
            break;

        case SOCKET_MSG_ACCEPT: { /* Called by listen() 
                                    when a remote TCP host is connected. */
            tstrSocketAcceptMsg *accept_msg = (tstrSocketAcceptMsg*)payload;
            SOCKET accepted_socket = accept_msg->sock;
            struct sockaddr_in remote_addr = accept_msg->strAddr;
            printf("Socket connecting request accepted :%d.\n", accepted_socket); 
            uint8_t *ip = (uint8_t *)&remote_addr.sin_addr.s_addr;
            printf("\t %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
            state_socket = 1;
        }
            break;

        case SOCKET_MSG_CONNECT: { /* Called from connect() */
            tstrSocketConnectMsg *connect_msg = (tstrSocketConnectMsg*)payload;
            // SOCKET connected_socket = connect_msg->sock;
            
            if (0 == connect_msg->s8Error) {
                /* Connection successful */
                puts("Socket connection is successful.");
                state_socket = 1;
            } else if (0 > connect_msg->s8Error) {
                /* Timeout or connect() is used with UDP socket */
                puts("Socket connection failed");
            } else {
                /* Unknown */
                puts("Unkown socket connection");
            }
        }
            break;

        case SOCKET_MSG_RECVFROM:   { /* Called from recvfrom(). For TCP/UDP */
            /* Only for SOCKET_MSG_RECVFROM */
            struct sockaddr_in remote_addr = ((tstrSocketRecvMsg*)payload)->strRemoteAddr;
            uint8_t *ip = (uint8_t *)&remote_addr.sin_addr.s_addr;
            uint16_t port = (uint16_t)((remote_addr.sin_port << 8)|(remote_addr.sin_port >> 8));
            printf("Socket[%d] Received from %u.%u.%u.%u:%u\n", sock, ip[0], ip[1], ip[2], ip[3], port);
        case SOCKET_MSG_RECV: ;        /* Called from recv(). For TCP/UDP */
            tstrSocketRecvMsg *recv_msg = (tstrSocketRecvMsg*)payload;
            // uint8_t *user_buf = recv_msg->pu8Buffer;
            int16_t size_received = recv_msg->s16BufferSize;
            uint16_t size_remaining = recv_msg->u16RemainingSize;

            printf("Socket[%d] %d bytes received. %d bytes remaining.\n", sock, size_received, size_remaining);
            
            switch (recv_msg->s16BufferSize) {
                case SOCK_ERR_NO_ERROR:	/* Socket connection closed */
                    puts("Socket closed while receiving.");
                    break;
                case SOCK_ERR_CONN_ABORTED:	/* Socket connection aborted */
                    puts("Socket connection aborted while receiving.");
                    break;
                case SOCK_ERR_TIMEOUT:	/* Socket recieve timed out */
                    puts("Socket recv timeout.");
                    break;
            }
            state_socket = 1;
        }
            break;
            
        case SOCKET_MSG_SEND: 		/* Called from send(). For TCP/UDP */
        case SOCKET_MSG_SENDTO:	{	/* Called from sendto(). For UDP */
            /* TODO: send() doesn't seem to trigger this event */
            if (*(int16_t*)payload < 1) {	/* Bytes sent */
                /* Sending error */
                printf("Sending failed :%d\n", *(int16_t*)payload);
            } else {
                printf("%d bytes sent\n", *(int16_t*)payload);
            }
            state_socket = 1;
        }
            break;
        default:
            printf("Unknown Socket message: %d\n", msg_type);
            break;
    }
}

extern struct sockaddr_in addr;
void winc1500_dns_resolve_cb(uint8_t* domain_name, uint32_t server_ip_addr)
{
    addr.sin_addr.s_addr = server_ip_addr;
    printf("DNS resolution successful: %s is:\n", domain_name);
    uint8_t *ip = (uint8_t *)&addr.sin_addr.s_addr;
    printf("\t %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
    state_socket = 1;
}