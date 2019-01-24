/**
******************************************************************************
  * @file    er-coap-dtls.c
  * @author  Central LAB
  * @version V1.0.0
  * @date    29-September-2017
  * @brief   CoAP-DTLS bindings
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "contiki-net.h"
#include "er-coap.h"
#include "er-coap-engine.h"
/*---------------------------------------------------------------------------*/
#if WITH_DTLS_COAP //closes at the end of the file
#include "platform-conf.h"
#include "er-coap-dtls.h"
#include "dtls.h"
#include <string.h>
#define DEBUG DEBUG_NONE
//#undef NDEBUG
#include "dtls_debug.h"
/*---------------------------------------------------------------------------*/
#include "keys.h" //Keys have been moved at Project level
/*---------------------------------------------------------------------------*/
#if DTLS_CONF_IDENTITY_HINT_SERIAL
extern unsigned char *serial_number;
#endif /*DTLS_CONF_IDENTITY_HINT_SERIAL*/
/*---------------------------------------------------------------------------*/
#define HIDENTITY_HINT_LEN 17
unsigned char identity_hint[HIDENTITY_HINT_LEN]="";
/*---------------------------------------------------------------------------*/
/**@def BYTE_ARRAY_INITIALIZER
 *
 * Initializes of existing byte array pointer to \a NULL.
 */
#undef BYTE_ARRAY_INITIALIZER
#define BYTE_ARRAY_INITIALIZER {NULL, 0}

typedef struct
{
    uint8_t *data;    /**< Pointer to the byte array */
    size_t len;      /**< Data size */
} byte_array;
/*---------------------------------------------------------------------------*/
#ifdef DTLS_X509
static const unsigned char x509_certificate[CERTIFICATE_LEN] = {CERTIFICATE_VALUE} ;
/* max len is DTLS_MAX_CERT_SIZE=1400*/

//default key pair
static const unsigned char x509_priv_key[]  = {X509_PRIV_KEY_VALUE} ;
static const unsigned char x509_pub_key_x[] = {X509_PUB_KEY_X_VALUE} ;
static const unsigned char x509_pub_key_y[] = {X509_PUB_KEY_Y_VALUE} ;

//default CA pub key
static const unsigned char x509_ca_pub_x[] = {X509_CA_PUB_X_VALUE} ;
static const unsigned char x509_ca_pub_y[] = {X509_CA_PUB_Y_VALUE} ;

#  if DTLS_ROLE_SERVER
/*We are Server, we need Client's Values*/
static const unsigned char client_pub_key_x[] = {X509_CLIENT_PUB_KEY_X_VALUE};
static const unsigned char client_pub_key_y[] = {X509_CLIENT_PUB_KEY_Y_VALUE};
#  else /*!DTLS_ROLE_SERVER -> DTLS_ROLE_CLIENT*/
/*We are Client, we need Server's Values*/
static const unsigned char server_pub_key_x[] =
	//{SERV_PUB_KEY_X_VALUE_LESHAN} ;
	{SERV_PUB_KEY_X_VALUE_DEFAULT};

static const unsigned char server_pub_key_y[] =
	//{SERV_PUB_KEY_Y_VALUE_LESHAN} ;
	{SERV_PUB_KEY_Y_VALUE_DEFAULT};
#  endif /*DTLS_ROLE_SERVER*/
#endif /*DTLS_X509*/
/*---------------------------------------------------------------------------*/
#ifdef DTLS_ECC
 static const unsigned char ecdsa_priv_key[] = {ECDSA_PRIV_KEY_VALUE} ;
 static const unsigned char ecdsa_pub_key_x[] = {ECDSA_PUB_KEY_X_VALUE} ;
 static const unsigned char ecdsa_pub_key_y[] = {ECDSA_PUB_KEY_Y_VALUE} ;
#endif /*DTLS_ECC*/
/*---------------------------------------------------------------------------*/
static int
send_to_peer(struct dtls_context_t *ctx,
             session_t *session, uint8 *data, size_t len);
static int
read_from_peer(struct dtls_context_t *ctx,
               session_t *session, uint8 *data, size_t len);
/*-----------------------------------------------------------------------------------*/
#ifdef DTLS_PSK
/* This function is the "key store" for tinyDTLS. It is called to
 * retrieve a key for the given identity within this particular
 * session. */
static int
get_psk_info(struct dtls_context_t *ctx, const session_t *session,
         dtls_credentials_type_t type,
         const unsigned char *id, size_t id_len,
         unsigned char *result, size_t result_length) {

  struct keymap_t {
    unsigned char *id;
    size_t id_length;
    unsigned char *key;
    size_t key_length;
  } psk[] = {
#if DTLS_CONF_IDENTITY_HINT_SERIAL
#  if DTLS_ROLE_SERVER
	/*For the moment, manually insert in the server the list of serial number of the client nodes... */
	{ (unsigned char *)"025134306D336F31", DTLS_IDENTITY_HINT_LENGTH, (unsigned char *)DTLS_PSK_KEY_VALUE, DTLS_PSK_KEY_VALUE_LENGTH },
	{ (unsigned char *)"0151343362355936", DTLS_IDENTITY_HINT_LENGTH, (unsigned char *)DTLS_PSK_KEY_VALUE, DTLS_PSK_KEY_VALUE_LENGTH },
#  elif DTLS_ROLE_CLIENT
	/*Identity Hint is node's serial number.*/
    { (unsigned char *)identity_hint, DTLS_IDENTITY_HINT_LENGTH, (unsigned char *)DTLS_PSK_KEY_VALUE, DTLS_PSK_KEY_VALUE_LENGTH },
#  endif /*DTLS_ROLE_SERVER || DTLS_ROLE_CLIENT*/
#else /*!DTLS_CONF_IDENTITY_HINT_SERIAL*/
	/*Default value, be sure that client and server side match.*/
	{ (unsigned char *)DTLS_IDENTITY_HINT, DTLS_IDENTITY_HINT_LENGTH, (unsigned char *)DTLS_PSK_KEY_VALUE, DTLS_PSK_KEY_VALUE_LENGTH },
#endif /*DTLS_CONF_IDENTITY_HINT_SERIAL*/
  };
  if (type ==  DTLS_PSK_IDENTITY) {
    if (id_len) {
      dtls_debug("got psk_identity_hint: '%.*s'\n", id_len, id);
    }

    if (result_length < psk[0].id_length) {
      dtls_warn("cannot set psk_identity -- buffer too small\n");
      return dtls_alert_fatal_create(DTLS_ALERT_INTERNAL_ERROR);
    }

    memcpy(result, psk[0].id, psk[0].id_length);
    return psk[0].id_length;
  } else if (type == DTLS_PSK_KEY) {
    if (id) {
      int i;
      for (i = 0; i < sizeof(psk)/sizeof(struct keymap_t); i++) {
        if (id_len == psk[i].id_length && memcmp(id, psk[i].id, id_len) == 0) {
          if (result_length < psk[i].key_length) {
            dtls_warn("buffer too small for PSK");
            return dtls_alert_fatal_create(DTLS_ALERT_INTERNAL_ERROR);
          }

          memcpy(result, psk[i].key, psk[i].key_length);
          return psk[i].key_length;
        }
      }
    }
  } else {
    return 0;
  }

  return dtls_alert_fatal_create(DTLS_ALERT_DECRYPT_ERROR);
}
#endif /* DTLS_PSK*/
/*-----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------*/
#ifdef DTLS_ECC
static int
get_ecdsa_key(struct dtls_context_t *ctx,
	      const session_t *session,
	      const dtls_ecdsa_key_t **result) {
  static const dtls_ecdsa_key_t ecdsa_key = {
    .curve = DTLS_ECDH_CURVE_SECP256R1,
    .priv_key = ecdsa_priv_key,
    .pub_key_x = ecdsa_pub_key_x,
    .pub_key_y = ecdsa_pub_key_y
  };

  PRINTF("GET_ECDSA_KEY\n");
  *result = &ecdsa_key;
  return 0;
}
/*-----------------------------------------------------------------------------------*/
static int
verify_ecdsa_key(struct dtls_context_t *ctx,
		 const session_t *session,
		 const unsigned char *other_pub_x,
		 const unsigned char *other_pub_y,
		 size_t key_size) {
  PRINTF("VERIFY_ECDSA_KEY\n");
  return 0;
}
#endif /* DTLS_ECC */
/*-----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------*/
#ifdef DTLS_X509
static int
get_x509_key(struct dtls_context_t *ctx,
          const session_t *session,
          const dtls_ecdsa_key_t **result) {
    (void)ctx;
    (void)session;
  static dtls_ecdsa_key_t ecdsa_key = {
    .curve = DTLS_ECDH_CURVE_SECP256R1,
    .priv_key = x509_priv_key,
    .pub_key_x = x509_pub_key_x,
    .pub_key_y = x509_pub_key_y
  };

  *result = &ecdsa_key;
  return 0;
}
/*-----------------------------------------------------------------------------------*/
static int
get_x509_cert(struct dtls_context_t *ctx,
        const session_t *session,
        const unsigned char **cert,
        size_t *cert_size)
{
    (void)ctx;
    (void)session;
	*cert = x509_certificate;
	*cert_size = CERTIFICATE_LEN;

    return 0;
}
/*-----------------------------------------------------------------------------------*/
int check_certificate(byte_array cert_der_code, byte_array ca_public_key)
{
    (void)cert_der_code;
    (void)ca_public_key;
    return 0;
}
/*-----------------------------------------------------------------------------------*/
static int verify_x509_cert(struct dtls_context_t *ctx, const session_t *session,
                                  const unsigned char *cert, size_t cert_size,
                                  unsigned char *x,
                                  size_t x_size,
                                  unsigned char *y,
                                  size_t y_size)
{
    int ret;
    const unsigned char *ca_pub_x;
    const unsigned char *ca_pub_y;
    byte_array cert_der_code = BYTE_ARRAY_INITIALIZER;
    byte_array ca_public_key = BYTE_ARRAY_INITIALIZER;
    unsigned char ca_pub_key_storage[DTLS_PUBLIC_KEY_SIZE];
    (void)ctx;
    (void)session;

	ca_pub_x = x509_ca_pub_x;
	ca_pub_y = x509_ca_pub_y;

    cert_der_code.data = (uint8_t *)cert;
    cert_der_code.len = cert_size;

    ca_public_key.len = DTLS_PUBLIC_KEY_SIZE;
    ca_public_key.data = ca_pub_key_storage;
    memcpy(ca_public_key.data, ca_pub_x, DTLS_PUBLIC_KEY_SIZE/2);
    memcpy(ca_public_key.data + DTLS_PUBLIC_KEY_SIZE/2, ca_pub_y, DTLS_PUBLIC_KEY_SIZE/2);

#if DTLS_ROLE_SERVER
    memcpy(x, client_pub_key_x, x_size);
    memcpy(y, client_pub_key_y, y_size);
#else /*!DTLS_ROLE_SERVER -> DTLS_ROLE_CLIENT*/
    memcpy(x, server_pub_key_x, x_size);
    memcpy(y, server_pub_key_y, y_size);
#endif /*DTLS_ROLE_SERVER*/

    ret = (int) check_certificate(cert_der_code, ca_public_key);

    return -ret;
}
/*-----------------------------------------------------------------------------------*/
static int is_x509_active(struct dtls_context_t *ctx)
{
    (void)ctx;
    return 0;
}
#endif /* DTLS_X509 */
/*-----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------*/
context_t *
coap_init_communication_layer(uint16_t port)
{
  static dtls_handler_t cb = {
    .write = send_to_peer,
    .read  = read_from_peer,
    .event = NULL,
#ifdef DTLS_PSK
    .get_psk_info = get_psk_info,
#endif /*DTLS_PSK*/
#ifdef DTLS_ECC
    .get_ecdsa_key = get_ecdsa_key,
    .verify_ecdsa_key = verify_ecdsa_key,
#endif /* DTLS_ECC*/
#ifdef DTLS_X509
	.get_x509_key = get_x509_key,
	.verify_x509_cert = verify_x509_cert,
	.get_x509_cert = get_x509_cert,
	.is_x509_active = is_x509_active,
#endif /* DTLS_X509 */
  };
  context_t * ctx;

#if DTLS_ROLE_CLIENT
#  if DTLS_CONF_IDENTITY_HINT_SERIAL
  NODE_UNIQUE_ID_CAP(identity_hint);
#    ifdef DTLS_PSK
  printf("Identity Hint for PSK (from serial number): '%s' (len %d).\n", identity_hint, strlen(identity_hint));
#    endif  /*DTLS_PSK*/
#  else /*!DTLS_CONF_IDENTITY_HINT_SERIAL*/
#    ifdef DTLS_PSK
  printf("Identity Hint for PSK (defined in project-conf.h): '%s' (len %d).\n", DTLS_IDENTITY_HINT, DTLS_IDENTITY_HINT_LENGTH);
#    endif  /*DTLS_PSK*/
#  endif /*DTLS_CONF_IDENTITY_HINT_SERIAL*/
#endif /*DTLS_ROLE_CLIENT*/

  struct uip_udp_conn *server_conn = udp_new(NULL, 0, NULL);
  udp_bind(server_conn, port);

  dtls_set_log_level(DTLS_LOG_DEBUG);

  ctx = dtls_new_context(server_conn);
  if(ctx) {
    dtls_set_handler(ctx, &cb);
  }
  /* new connection with remote host */
  printf("COAP-DTLS listening on port %u\n", uip_ntohs(server_conn->lport));
  return ctx;
}
/*-----------------------------------------------------------------------------------*/
static int
send_to_peer(struct dtls_context_t *ctx,
             session_t *session, uint8 *data, size_t len)
{
  struct uip_udp_conn *conn = (struct uip_udp_conn *)dtls_get_app_data(ctx);

  uip_ipaddr_copy(&conn->ripaddr, &session->addr);
  conn->rport = session->port;

  uip_udp_packet_send(conn, data, len);

  /* Restore server connection to allow data from any node */
  memset(&conn->ripaddr, 0, sizeof(conn->ripaddr));
  memset(&conn->rport, 0, sizeof(conn->rport));

  return len;
}
/*-----------------------------------------------------------------------------------*/
void
coap_send_message(context_t * ctx, uip_ipaddr_t *addr, uint16_t port, uint8_t *data, uint16_t length)
{
  session_t session;

  dtls_session_init(&session);
  uip_ipaddr_copy(&session.addr, addr);
  session.port = port;

  dtls_write(ctx, &session, data, length);
}
/*-----------------------------------------------------------------------------------*/
static int
read_from_peer(struct dtls_context_t *ctx,
               session_t *session, uint8 *data, size_t len)
{
  uip_len = len;
  memmove(uip_appdata, data, len);
  coap_receive(ctx);
  return 0;
}
/*-----------------------------------------------------------------------------------*/
void
coap_handle_receive(context_t *ctx)
{
  session_t session;

  if(uip_newdata()) {
    dtls_session_init(&session);
    uip_ipaddr_copy(&session.addr, &UIP_IP_BUF->srcipaddr);
    session.port = UIP_UDP_BUF->srcport;

    dtls_handle_message(ctx, &session, uip_appdata, uip_datalen());
  }
}
#endif /*WITH_DTLS_COAP*/
