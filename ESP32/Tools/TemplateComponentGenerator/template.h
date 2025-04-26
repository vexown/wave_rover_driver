/******************************************************************************
 * @file template.h
 * @brief Header file for the Template component
 *
 ******************************************************************************/

#ifndef TEMPLATE_H
#define TEMPLATE_H

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/*     Include headers required for the declarations in *this* header file     */
/*                 (e.g., types used in function prototypes).                  */
/*       Prefer forward declarations over full includes where possible         */
/*             to minimize dependencies (for structs, enums etc.).             */
/*******************************************************************************/

/* C Standard Libraries */
#include <stdint.h>
#include <stdbool.h> 

/* ESP-IDF Libraries */
#include "esp_err.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/
/*   for variables defined in the corresponding .c file, for use in other .c   */
/*      files just by including this header file. Use extern for these.        */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*   for functions defined in the corresponding .c file, for use in other .c   */
/*    files just by including this header file. Extern is a default linkage    */
/*    specifier for functions, so it is not necessary to use it explicitly.    */
/*******************************************************************************/

/**
 * @brief Perform an action using the Template component.
 *
 * @details This is an example of to document a function that is shared across
 *          multiple files. Keep to the convention of using Doxygen style and
 *          place them above the function declaration.
 *
 * @param[in] input_data Input data for the action.
 * @param[out] output_data Pointer to store the result.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if input parameters are invalid
 *      - Other esp_err_t codes as needed
 */
// esp_err_t template_perform_action(int input_data, int* output_data);

#endif /* TEMPLATE_H */