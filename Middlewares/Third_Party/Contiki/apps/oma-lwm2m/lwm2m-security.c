/*
 * Copyright (c) 2015, Yanzi Networks AB.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \addtogroup oma-lwm2m
 * @{
 *
 */

/**
 * \file
 *         Implementation of the Contiki OMA LWM2M security
 * \author
 *         Joakim Eriksson <joakime@sics.se>
 *         Niclas Finne <nfi@sics.se>
 */

#include <stdint.h>
#include "lwm2m-object.h"
#include "lwm2m-engine.h"
#if WITH_DTLS_COAP
#include "dtls.h"
#include "er-coap-dtls.h"
#endif

#include "keys.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef LWM2M_CONF_SERVER_MAX_COUNT
#define MAX_COUNT LWM2M_CONF_SERVER_MAX_COUNT
#else
#define MAX_COUNT 1 //2 MGR
#endif

#if WITH_DTLS_COAP && DTLS_CONF_IDENTITY_HINT_SERIAL
extern unsigned char identity_hint[];
#endif

#ifdef DTLS_X509
#define MAX_SIZE DTLS_MAX_CERT_SIZE
#elif defined (DTLS_ECC)
#define MAX_SIZE 64+2
#elif defined (DTLS_PSK)
#define MAX_SIZE  DTLS_PSK_MAX_KEY_LEN
#else
#define MAX_SIZE 64
#endif /*SECURITY MODE*/

static int32_t bs_arr[MAX_COUNT] = {0};

static int32_t secmode_arr[MAX_COUNT] =
#ifdef DTLS_X509
	{2} ;
#elif defined (DTLS_ECC)
	{1};
#elif defined (DTLS_PSK)
	{0};
#else
	{3}; //NO SEC
#endif /*SECURITY MODE*/

static int32_t sid_arr[MAX_COUNT] = {101} ;

static char server_uri[MAX_COUNT][MAX_SIZE] = {""};//FIXME
static uint16_t su_len[MAX_COUNT] = {0};

static /*const*/ unsigned char client_id[MAX_COUNT][MAX_SIZE] =
#ifdef DTLS_X509
{{CLIENT_CERTIFICATE_VALUE}} ;
#elif defined (DTLS_ECC)
{{0x00, 0x02, ECDSA_PUB_KEY_X_VALUE, ECDSA_PUB_KEY_Y_VALUE}}; //probably wrong, to be updated to SubjectPublicKeyInfo structure
#elif defined (DTLS_PSK)
# if !(DTLS_CONF_IDENTITY_HINT_SERIAL)
{{DTLS_IDENTITY_HINT}};
# else
{{0x00}};//To be set in the initialization since it can be not constant
# endif
#else
{{0x00}};
#endif /*SECURITY METHOD*/
static uint16_t client_id_len[MAX_COUNT] = {sizeof(client_id[0])};

static const unsigned char server_id[MAX_COUNT][MAX_SIZE] =
#ifdef DTLS_ECC
{{0x00, 0x02, SERV_PUB_KEY_X_VALUE_LESHAN, SERV_PUB_KEY_Y_VALUE_LESHAN}};  //probably wrong, to be updated to SubjectPublicKeyInfo structure
#else
{{0x00}};
#endif /*DTLS_ECC*/
static uint16_t server_id_len[MAX_COUNT] = {sizeof(server_id[0])};

static const unsigned char psk_key[MAX_COUNT][MAX_SIZE]; //TODO For the moment I do not put the secret key here
static uint16_t psk_key_len[MAX_COUNT] = {0};


static lwm2m_instance_t security_instances[MAX_COUNT];

static int
read_uri(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
    printf("*** read_uri\n");
	return ctx->writer->write_string(ctx, outbuf, outsize,
										server_uri[0], su_len[0]);
}

static int
read_bs(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
    printf("*** read_bs\n");
    return ctx->writer->write_int(ctx, outbuf, outsize,
                                    bs_arr[0]);
}

static int
read_sm(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
    printf("*** read_sm\n");
    return ctx->writer->write_int(ctx, outbuf, outsize,
    							secmode_arr[0]);
}

static int
read_pk(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
    printf("*** read_pk\n");
	return ctx->writer->write_string(ctx, outbuf, outsize,
									client_id[0], client_id_len[0]);
}

static int
read_spk(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
    printf("*** read_spk\n");
	return ctx->writer->write_string(ctx, outbuf, outsize,
										server_id[0], server_id_len[0]);
}

static int
read_sk(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
    printf("*** read_sk\n");
	return ctx->writer->write_string(ctx, outbuf, outsize,
										psk_key[0], psk_key_len[0]);
}

static int
read_ssid(lwm2m_context_t *ctx, uint8_t *outbuf, size_t outsize)
{
    printf("*** read_ssid\n");
    return ctx->writer->write_int(ctx, outbuf, outsize,
    								sid_arr[0]);
}

LWM2M_RESOURCES(security_resources,
				LWM2M_RESOURCE_CALLBACK(0, { read_uri, NULL, NULL }),
				LWM2M_RESOURCE_CALLBACK(1, { read_bs, NULL, NULL }),
				LWM2M_RESOURCE_CALLBACK(2, { read_sm, NULL, NULL }),
				LWM2M_RESOURCE_CALLBACK(3, { read_pk, NULL, NULL }),
				LWM2M_RESOURCE_CALLBACK(4, { read_spk, NULL, NULL }),
				LWM2M_RESOURCE_CALLBACK(5, { read_sk, NULL, NULL }),
				LWM2M_RESOURCE_CALLBACK(10, { read_ssid, NULL, NULL })
				);
/*
LWM2M_RESOURCES(security_resources,
                LWM2M_RESOURCE_STRING_VAR_ARR(0, MAX_COUNT, MAX_SIZE, su_len, server_uri),
                LWM2M_RESOURCE_INTEGER_VAR_ARR(1, MAX_COUNT, bs_arr),
                LWM2M_RESOURCE_INTEGER_VAR_ARR(2, MAX_COUNT, secmode_arr),
                LWM2M_RESOURCE_STRING_VAR_ARR(3, MAX_COUNT, MAX_SIZE, client_id_len, client_id),
                LWM2M_RESOURCE_STRING_VAR_ARR(4, MAX_COUNT, MAX_SIZE, server_id_len, server_id),
                LWM2M_RESOURCE_STRING_VAR_ARR(5, MAX_COUNT, MAX_SIZE, psk_key_len, psk_key),
                LWM2M_RESOURCE_INTEGER_VAR_ARR(10, MAX_COUNT, sid_arr)
                );*/
LWM2M_OBJECT(security, 0, security_instances);
/*---------------------------------------------------------------------------*/
void
lwm2m_security_init(void)
{
  //For the moment I leave "UNUSED" so it is not pushed to server, as per LWM2M spec
  lwm2m_instance_t template = LWM2M_INSTANCE_UNUSED(0, security_resources);
  int i;

  /* Initialize the instances */
  for(i = 0; i < MAX_COUNT; i++) {
    security_instances[i] = template;
    security_instances[i].id = i;
  }
#if WITH_DTLS_COAP && DTLS_CONF_IDENTITY_HINT_SERIAL
  memcpy (&(client_id[MAX_COUNT][0]), identity_hint, DTLS_CONF_IDENTITY_HINT_LENGTH);
#endif

  /**
   * Register this device and its handlers - the handlers
   * automatically sends in the object to handle.
   */
  PRINTF("*** Init lwm2m-security\n");
  lwm2m_engine_register_object(&security);
}
/*---------------------------------------------------------------------------*/
/** @} */
