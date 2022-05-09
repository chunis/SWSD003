/*!
 * @file      main_tx_infinite_preamble.c
 *
 * @brief     Example of the transmission of an infinite preamble for LR11xx chip
 *
 * @copyright
 * Revised BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdio.h>
#include <string.h>

#include "apps_common.h"
#include "apps_utilities.h"
#include "lr11xx_radio.h"
#include "smtc_board.h"
#include "smtc_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define APP_PARTIAL_SLEEP true

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lr11xx_hal_context_t* context;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    hal_mcu_disable_irq( );

    /* Init board */
    hal_mcu_init( );
    smtc_board_init_periph( );

    hal_mcu_enable_irq( );

    HAL_DBG_TRACE_INFO( "===== LR11xx TX infinite preamble example =====\n\n" );

    /* Configure the partial low power mode */
    hal_mcu_partial_sleep_enable( APP_PARTIAL_SLEEP );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( ( void* ) context );

    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );

    apps_common_lr11xx_radio_init( ( void* ) context );

    smtc_shield_handle_pre_tx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx_infinite_preamble( context ) );

    // Reload watchdog counter to avoid reset
    while( 1 )
    {
#if( HAL_USE_WATCHDOG == HAL_FEATURE_ON )
        hal_watchdog_reload( );
#endif  // HAL_USE_WATCHDOG == HAL_FEATURE_ON

        hal_mcu_set_sleep_for_ms( 100 );
    }
}