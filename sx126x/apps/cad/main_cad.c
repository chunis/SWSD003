/*!
 * @file      main_cad.c
 *
 * @brief     Channel Activity Detection (CAD) example for Sx126x chip
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdio.h>
#include <string.h>

#include "apps_common.h"
#include "apps_utilities.h"

#include "sx126x.h"
#include "main_cad.h"
#include "sx126x_str.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define DOUBLE_CAD_MODE
#define SINGLE_CAD_SYMBAL_NUM SX126X_CAD_01_SYMB
#define DOUBLE_CAD_SYMBAL_NUM SX126X_CAD_02_SYMB

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static sx126x_hal_context_t* context;

static sx126x_cad_params_t cad_params = {
    .cad_symb_nb     = SINGLE_CAD_SYMBAL_NUM,
    .cad_detect_peak = CAD_DETECT_PEAK,
    .cad_detect_min  = CAD_DETECT_MIN,
    .cad_exit_mode   = CAD_EXIT_MODE,
    .cad_timeout     = 0,
};

static uint8_t  buffer[PAYLOAD_LENGTH];
static uint32_t iteration_number        = 0;
static uint32_t received_packet_counter = 0;
static uint32_t detection_counter       = 0;

static uint32_t all = 0;
static uint32_t detect = 0;
static uint32_t lost = 0;
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void start_cad_after_delay( uint16_t delay_ms );
static void start_cad_after_delay_us( uint16_t delay_us );
static uint32_t delay_n_symbols_us(uint32_t n, sx126x_lora_sf_t sf);

static void optimize_cad_parameters( sx126x_lora_sf_t sf, sx126x_cad_params_t* cad_params );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    smtc_hal_mcu_init( );
    uart_init( );

    HAL_DBG_TRACE_INFO( "===== SX126x CAD example =====\n\n" );
    apps_common_sx126x_print_version_info( );
    HAL_DBG_TRACE_INFO( "CAD Exit Mode : %s\n", sx126x_cad_exit_modes_to_str( CAD_EXIT_MODE ) );

#ifdef DOUBLE_CAD_MODE
    HAL_DBG_TRACE_PRINTF( "Double CAD mode\n" );
#else
    HAL_DBG_TRACE_PRINTF( "Single CAD mode\n" );
#endif

    HAL_DBG_TRACE_PRINTF( "Freq: %d\n", RF_FREQ_IN_HZ );
    HAL_DBG_TRACE_PRINTF( "SF: %d\n", LORA_SPREADING_FACTOR );
    HAL_DBG_TRACE_PRINTF( "RX: BOOST\n" );
    HAL_DBG_TRACE_PRINTF( "REF: TCXO\n" );
    //HAL_DBG_TRACE_PRINTF( "CAD_SYMBOL_NUM: %d\n", CAD_SYMBOL_NUM );
#ifdef DOUBLE_CAD_MODE
    HAL_DBG_TRACE_PRINTF( "CAD_SYMBOL_NUM: (%d, %d)\n", 1 << SINGLE_CAD_SYMBAL_NUM, 1 << DOUBLE_CAD_SYMBAL_NUM );
#else
    HAL_DBG_TRACE_PRINTF( "CAD_SYMBOL_NUM: %d\n", 1 << SINGLE_CAD_SYMBAL_NUM );
#endif
    HAL_DBG_TRACE_PRINTF( "CODING_RATE: %d\n", LORA_CODING_RATE );
    HAL_DBG_TRACE_PRINTF( "CAD_DETECT_PEAK: %d\n", CAD_DETECT_PEAK );
    HAL_DBG_TRACE_PRINTF( "CAD_DETECT_MIN: %d\n\n", CAD_DETECT_MIN );


    apps_common_shield_init( );
    context = apps_common_sx126x_get_context( );

    apps_common_sx126x_init( ( void* ) context );
    apps_common_sx126x_radio_init( ( void* ) context );

    sx126x_set_dio_irq_params( context, SX126X_IRQ_ALL,
                               SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_CAD_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE |
                                   SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_CRC_ERROR,
                               SX126X_IRQ_NONE, SX126X_IRQ_NONE );

    sx126x_clear_irq_status( context, SX126X_IRQ_ALL );

    optimize_cad_parameters( LORA_SPREADING_FACTOR, &cad_params );

    if( cad_params.cad_exit_mode == SX126X_CAD_RX )
    {
        cad_params.cad_timeout = sx126x_convert_timeout_in_ms_to_rtc_step( CAD_TIMEOUT_MS );
    }
    else if( cad_params.cad_exit_mode == SX126X_CAD_LBT )
    {
        for( int i = 0; i < PAYLOAD_LENGTH; i++ )
        {
            buffer[i] = i;
        }
        ASSERT_SX126X_RC( sx126x_write_buffer( context, 0, buffer, PAYLOAD_LENGTH ) );
    }
    ASSERT_SX126X_RC( sx126x_set_cad_params( context, &cad_params ) );

    start_cad_after_delay( DELAY_MS_BEFORE_CAD );

    while( 1 )
    {
        apps_common_sx126x_irq_process( ( void* ) context );
    }
}

bool detect_first = false;

void on_cad_done_detected( void )
{
    all++;

#ifdef DOUBLE_CAD_MODE
    //HAL_DBG_TRACE_PRINTF( "cad detected\n" );
    if(detect_first == false){
        detect_first = true;
    } else {  // detect for the 2nd time
        detect_first = false;
        detect++;
        HAL_DBG_TRACE_PRINTF( "  DETECTED,all=%d,detect=%d,lost=%d\n", all, detect, lost );
    }
#else
    detect++;
    HAL_DBG_TRACE_PRINTF( "  DETECTED,all=%d,detect=%d,lost=%d\n", all, detect, lost );
#endif

    detection_counter++;
    //HAL_DBG_TRACE_INFO( "Consecutive detection(s): %d\n", detection_counter );
    switch( cad_params.cad_exit_mode )
    {
    case SX126X_CAD_ONLY:
        //HAL_DBG_TRACE_INFO( "Switch to StandBy mode\n" );
#ifdef DOUBLE_CAD_MODE
        if(detect_first == false){
            cad_params.cad_symb_nb     = SINGLE_CAD_SYMBAL_NUM;
            ASSERT_SX126X_RC( sx126x_set_cad_params( context, &cad_params ) );
            start_cad_after_delay( DELAY_MS_BEFORE_CAD );
        } else {
            cad_params.cad_symb_nb     = DOUBLE_CAD_SYMBAL_NUM;
            //cad_params.cad_detect_peak     = CAD_DETECT_PEAK;  // here can change to a different peak value
            ASSERT_SX126X_RC( sx126x_set_cad_params( context, &cad_params ) );
            start_cad_after_delay_us( delay_n_symbols_us( 1, LORA_SPREADING_FACTOR ) );
        }
#else
        start_cad_after_delay( DELAY_MS_BEFORE_CAD );
#endif
        break;
    case SX126X_CAD_RX:
        HAL_DBG_TRACE_INFO( "Switch to RX mode\n" );
        apps_common_sx126x_handle_pre_rx( );
        break;
    case SX126X_CAD_LBT:
        start_cad_after_delay( DELAY_MS_BEFORE_CAD );
        break;
    default:
        HAL_DBG_TRACE_ERROR( "Unknown CAD exit mode: 0x%02x\n", cad_params.cad_exit_mode );
        break;
    }
}

void on_cad_done_undetected( void )
{
    //HAL_DBG_TRACE_PRINTF( "cad undetected\n" );
    all++;
    lost++;

#ifdef DOUBLE_CAD_MODE
    detect_first = false;
#endif

    HAL_DBG_TRACE_PRINTF( "UNDETECTED,all=%d,detect=%d,lost=%d\n", all, detect, lost );

    detection_counter = 0;
    switch( cad_params.cad_exit_mode )
    {
    case SX126X_CAD_ONLY:
        //HAL_DBG_TRACE_INFO( "Switch to StandBy mode\n" );
#ifdef DOUBLE_CAD_MODE
        cad_params.cad_symb_nb     = SINGLE_CAD_SYMBAL_NUM;
        ASSERT_SX126X_RC( sx126x_set_cad_params( context, &cad_params ) );
#endif
        start_cad_after_delay( DELAY_MS_BEFORE_CAD );
        break;
    case SX126X_CAD_RX:
        start_cad_after_delay( DELAY_MS_BEFORE_CAD );
        break;
    case SX126X_CAD_LBT:
        HAL_DBG_TRACE_INFO( "Switch to TX mode\n" );
        apps_common_sx126x_handle_pre_tx( );
        break;
    default:
        HAL_DBG_TRACE_ERROR( "Unknown CAD exit mode: 0x%02x\n", cad_params.cad_exit_mode );
        break;
    }
}

void on_tx_done( void )
{
    apps_common_sx126x_handle_post_tx( );
    buffer[0] = buffer[0] + 1;
    ASSERT_SX126X_RC( sx126x_write_buffer( context, 0, buffer, PAYLOAD_LENGTH ) );
    start_cad_after_delay( DELAY_MS_BEFORE_CAD );
}

void on_rx_done( void )
{
    uint8_t size;
    memset( buffer, 0, PAYLOAD_LENGTH );
    apps_common_sx126x_handle_post_rx( );
    apps_common_sx126x_receive( ( void* ) context, buffer, &size, PAYLOAD_LENGTH );
    HAL_DBG_TRACE_INFO( "Consecutive reception(s): %d\n", received_packet_counter );
    received_packet_counter++;
    HAL_DBG_TRACE_ARRAY( "Received packet: ", buffer, PAYLOAD_LENGTH );
    start_cad_after_delay( DELAY_MS_BEFORE_CAD );
}

void on_rx_timeout( void )
{
    apps_common_sx126x_handle_post_rx( );
    start_cad_after_delay( DELAY_MS_BEFORE_CAD );
    received_packet_counter = 0;
}

void on_rx_error( void )
{
    apps_common_sx126x_handle_post_rx( );
    start_cad_after_delay( DELAY_MS_BEFORE_CAD );
    received_packet_counter = 0;
}

static void start_cad_after_delay( uint16_t delay_ms )
{
    //HAL_DBG_TRACE_PRINTF( "\nStart CAD, iteration %d\n", iteration_number++ );
    LL_mDelay( delay_ms );
    ASSERT_SX126X_RC( sx126x_set_cad( context ) );
}

void __attribute__( ( optimize( "O0" ) ) ) hal_mcu_wait_us( const int32_t microseconds )
{
    // Work @80MHz
    const uint32_t nb_nop = microseconds * 1000 / 137;
    for( uint32_t i = 0; i < nb_nop; i++ )
    {
        __NOP( );
    }
}

#ifdef DOUBLE_CAD_MODE
static void start_cad_after_delay_us( uint16_t delay_us )
{
    //HAL_DBG_TRACE_PRINTF( "Start CAD\n" );
    hal_mcu_wait_us( delay_us );
    ASSERT_SX126X_RC( sx126x_set_cad( context ) );
    //HAL_DBG_TRACE_PRINTF( "\n" );  // Empty line to split CAD info display
}
#endif

static uint32_t delay_n_symbols_us(uint32_t n, sx126x_lora_sf_t sf)
{
    return (1024 << (sf - SX126X_LORA_SF7)) * n;
}

#if USER_PROVIDED_CAD_PARAMETERS == False
static void optimize_cad_parameters( sx126x_lora_sf_t sf, sx126x_cad_params_t* cad_params )
{
    switch( sf )
    {
    case SX126X_LORA_SF7:
        cad_params->cad_detect_min  = 10;
        cad_params->cad_detect_peak = 22;
        cad_params->cad_symb_nb     = 2;
        break;
    case SX126X_LORA_SF8:
        cad_params->cad_detect_min  = 10;
        cad_params->cad_detect_peak = 22;
        cad_params->cad_symb_nb     = 2;
        break;
    case SX126X_LORA_SF9:
        cad_params->cad_detect_min  = 10;
        cad_params->cad_detect_peak = 23;
        cad_params->cad_symb_nb     = 4;
        break;
    case SX126X_LORA_SF10:
        cad_params->cad_detect_min  = 10;
        cad_params->cad_detect_peak = 24;
        cad_params->cad_symb_nb     = 4;
        break;
    case SX126X_LORA_SF11:
        cad_params->cad_detect_min  = 10;
        cad_params->cad_detect_peak = 25;
        cad_params->cad_symb_nb     = 4;
        break;
    default:
        HAL_DBG_TRACE_WARNING( "CAD may not function properly while using these radio parameters\n" );
        break;
    }
}
#else
static void optimize_cad_parameters( sx126x_lora_sf_t sf, sx126x_cad_params_t* cad_params )
{
}
#endif
