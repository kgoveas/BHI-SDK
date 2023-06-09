/* 
 * $license$
 */

/*! @addtogroup bsx
 * @{
 */

#ifndef BSX_RETURN_VALUE_IDENTIFIER_H_
#define BSX_RETURN_VALUE_IDENTIFIER_H_

/*! @brief Provides the enumeration containing return value identifier 
 *
 * @{
 */ 
/** @brief Normal operation was successful without any warning or additional information. */
#define BSX_OK   (0)

/** @brief Invalid input of doSteps. */
#define BSX_E_DOSTEPS_INVALIDINPUT   (-1)

/** @brief Value limits are exceeded. */
#define BSX_E_DOSTEPS_VALUELIMITS   (-2)

/** @brief Requested virtual sensor output is disabled. */
#define BSX_E_GET_DISABLED   (-3)

/** @brief No data is returned by get. */
#define BSX_E_GET_NODATA   (-4)

/** @brief Difference of timestamps between subsequent doSteps calls is out of range. */
#define BSX_E_DOSTEPS_TSINTRADIFFOUTOFRANGE   (-5)

/** @brief Input with same physical sensor identifier is given multiple times. */
#define BSX_E_DOSTEPS_DUPLICATEINPUT   (-6)

/** @brief Difference of timestamps for multiples sensors at same doStep call is out of range. */
#define BSX_E_DOSTEPS_TSINTERDIFFOUTOFRANGE   (-7)

/** @brief No outputs are returned by doSteps because no results processed. */
#define BSX_I_DOSTEPS_NOOUTPUTSRETURNED   (1)

/** @brief No outputs are returned by doSteps because no memory provided. */
#define BSX_I_DOSTEPS_NOOUTPUTSRETURNABLE   (2)

/** @brief More outputs can be returned but not enough memory provided. */
#define BSX_W_DOSTEPS_EXCESSOUTPUTS   (3)

#define BSX_I_DOSTEPS_NOINPUTDATA   (4)

#define BSX_I_DOSTEPS_NOTHREADLOAD   (5)

/** @brief Invalid sample rate was given (sample rate for event based input, or event enabled for sample rate based). */
#define BSX_E_SU_WRONGDATARATE   (-10)

/** @brief Maximum number of iterations for update subscription reached. */
#define BSX_E_SU_ITERATIONSEXCEEDED   (-11)

/** @brief Sample rate is out of sample rate limits. */
#define BSX_E_SU_SAMPLERATELIMITS   (-12)

/** @brief Duplicate usage of virtual sensor in updateSubscription. */
#define BSX_E_SU_DUPLICATEGATE   (-13)

/** @brief Invalid sample rate was given to updateSubscription. */
#define BSX_E_SU_INVALIDSAMPLERATE   (-14)

/** @brief Number of configured virtual sensor exceeds number of outputs.  */
#define BSX_E_SU_GATECOUNTEXCEEDSARRAY   (-15)

/** @brief Unknown identifier for virtual sensor given to updateSubscription. */
#define BSX_W_SU_UNKNOWNOUTPUTGATE   (10)

/** @brief No output was given to updateSubscription. */
#define BSX_W_SU_NOOUTPUTGATE   (11)

/** @brief Subscribed output gates have been returned. */
#define BSX_I_SU_SUBSCRIBEDOUTPUTGATES   (12)

/** @brief Subscribed input gates have been returned. */
#define BSX_I_SU_SUBSCRIBEDINPUTGATES   (13)

/** @brief indicates the change in supple sample intervals */
#define BSX_I_SU_CORRECTIONOFBOUNDS   (15)

/** @brief Running out of instances. Library initialized too often. */
#define BSX_E_INSTANCE_COUNT_EXCEEDED   (-250)

/** @brief Disables specific gates for particular mode */
#define BSX_E_SU_OUTPUTDISABLED   (-16)

/** @brief Operation aborted.  */
#define BSX_E_ABORT   (-252)

/** @brief Library is in invalid state. Operation is not allowed. */
#define BSX_E_INVALIDSTATE   (-253)

/** @brief Fatal error. */
#define BSX_E_FATAL   (-254)

/** @brief Set-up failed. (deprecated) */
#define BSX_E_SETUP_FAIL   (-255)

/** @brief Length of section exceeds work buffer. */
#define BSX_E_PARSE_SECTIONEXCEEDSWORKBUFFER   (-32)

/** @brief Configuration failed. */
#define BSX_E_CONFIG_FAIL   (-33)

/** @brief Version of serialization does not match version of library. */
#define BSX_E_CONFIG_VERSIONMISMATCH   (-34)

/** @brief Feature set of serialization does not match features set of library. */
#define BSX_E_CONFIG_FEATUREMISMATCH   (-35)

/** @brief CRC of serialization is invalid. */
#define BSX_E_CONFIG_CRCMISMATCH   (-36)

/** @brief Serialization is empty. */
#define BSX_E_CONFIG_EMPTY   (-37)

/** @brief Insufficient work buffer for serialization. */
#define BSX_E_CONFIG_INSUFFICIENTWORKBUFFER   (-38)

/** @brief Invalid number of sections in serialization. */
#define BSX_E_CONFIG_INVALIDSECTIONCOUNT   (-39)

/** @brief Invalid length of serialization. */
#define BSX_E_CONFIG_INVALIDSTRINGSIZE   (-40)

/** @brief Insufficient buffer for serialization. */
#define BSX_E_CONFIG_INSUFFICIENTBUFFER   (-41)

/** @brief Serialized configuration/state is too large. */
#define BSX_E_CONFIG_STRINGTOOLARGE   (-42)

/** @brief Invalid channel identifier used in setting of configuration or state. */
#define BSX_E_SET_INVALIDCHANNELIDENTIFIER   (-100)

/** @brief Invalid property identifier used in setting of configuration or state. */
#define BSX_E_SET_INVALIDPROPERTY   (-101)

/** @brief Invalid value. */
#define BSX_E_SET_INVALIDVALUE   (-103)

/** @brief Invalid length of serialization. */
#define BSX_E_SET_INVALIDLENGTH   (-104)

/** @brief Insufficient buffer for serialization. */
#define BSX_E_DUMP_INSUFFICIENTBUFFER   (-200)

/*!
 * @}
 */ 

#endif /* BSX_RETURN_VALUE_IDENTIFIER_H_ */

/*! @}
 */
