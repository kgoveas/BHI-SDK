 /*
 * $license$
 */

/*!@addtogroup bsx_external
 * @{*/

#ifndef __BSX_DATATYPES_H__
#define __BSX_DATATYPES_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*! @brief includes */
#include <stddef.h>
#include <stdint.h>


#define BSX_FALSE (0)        /**< defines the use of false  */
#define BSX_TRUE (1)        /**< defines the use of true  */


/*! @brief structure definition to hold the version information of the software */
typedef struct
{
    uint8_t version_major; /**< major version */
    uint8_t version_minor; /**< minor version */
    uint8_t bugfix_major; /**< major bug-fix identification */
    uint8_t bugfix_minor; /**< minor bug-fix identification */
} bsx_version_t;

/*!
 * @brief Definition of the pointer to the BSX4 library
 */
typedef size_t bsx_instance_t;


/*!
 * @brief Content of one dimension of a frame in a FIFO data set
 * \note Supports only 32 bit values.
 */
typedef union
{
    float sfp; /**< floating-point access  */
    int32_t sli; /**< signed integer access */
    uint32_t uli; /**< unsigned integer access */
    struct
    {
        int16_t value; /**< signed integer value */
        /* 2 bytes remaining for future usage */
    } ssi; /**< signed short integer access */
    struct
    {
        uint16_t value; /**< unsigned integer value */
        /* 2 bytes remaining for future usage */
    } usi; /**< unsigned short integer access */
    struct
    {
        int8_t value; /**< signed integer value */
        /* 3 bytes remaining for future usage */
    } sti; /**< signed short integer access */
    struct
    {
        uint8_t value; /**< unsigned integer value */
        /* 3 bytes remaining for future usage */
    } uti; /**< unsigned short integer access */
} bsx_data_content_t;


/*! @brief BSX specific return value */
typedef int32_t bsx_return_t;


/*!
 * @brief Storage for a FIFO data set using frames
 *
 * One frame consists of \p nDim values. A set consists of \p nLength frames.
 */
typedef struct
{
    int32_t reserved;       /**< not used */
    int32_t time_stamp;     /**< time stamp */
    bsx_data_content_t *content_p; /**< content of the FIFO set */
    int32_t data_type;      /**< data type used within the frames of the FIFO set */
    uint16_t sensor_id;     /**< identifier of the virtual or physical sensor  */
    uint8_t dims;           /**< number of dimensions in a frame  */
    uint8_t depth;          /**< number of frames in a FIFO set */
    int32_t padding;        /**< padding for certain compilers which pad struct to a multiple of the biggest member */
} bsx_fifo_data_x32_t;

/*!
 * @brief Storage for a FIFO data set for x64 interface using frames
 *
 * One frame consists of \p nDim values. A set consists of \p nLength frames.
 */
typedef struct
{
    int64_t time_stamp;     /**< time stamp */
    bsx_data_content_t *content_p; /**< content of the FIFO set */
    int32_t data_type;      /**< data type used within the frames of the FIFO set */
    uint16_t sensor_id;     /**< identifier of the virtual or physical sensor  */
    uint8_t dims;           /**< number of dimensions in a frame  */
    uint8_t depth;          /**< number of frames in a FIFO set */
    int32_t padding;        /**< padding for certain compilers which pad struct to a multiple of the biggest member */
} bsx_fifo_data_t;

/*! @brief Configuration of a virtual or physical sensor */
typedef struct
{
    uint32_t sensor_id;                /*!< identifier of the virtual or physical sensor */
    float sample_rate;              /*!< sample rate of the virtual or physical sensor */
} bsx_sensor_configuration_t;

/*! @brief Characteristics of a sensor */
typedef struct
{
    uint32_t sensor_id;           /*!< identifier of the virtual sensor */
    float sample_rate_current;    /*!< current sample rate of the virtual sensor */
    float sample_rate_max;        /*!< max sample rate of the virtual sensor */
    float sample_rate_min;        /*!< min sample rate of the virtual sensor */
    uint16_t dependencies;        /*!< dependency bitfield (required inputs) */
} bsx_sensor_characteristics_t;


/*! @brief Create one function local buffer
 *
 * @note If used within a function, the stack size may be increased tremendously depending on the
 *       dimensions and frames. Use \p BSX_CREATE_FIFO_GLOBAL to allocate globally in RAM instead
 *       of putting it on the stack.
 */
#define BSX_CREATE_FIFO_LOCAL(name, type, n_dims, n_frames)  \
bsx_data_content_t bsx_##name##_buffer[(n_dims)*(n_frames)]; \
bsx_fifo_data_t bsx_##name =                                 \
{                                                            \
        0,                                                   \
        (bsx_##name##_buffer),                               \
        (type),                                              \
        0U,                                                  \
        (n_dims),                                            \
        (n_frames),                                          \
        0U,                                                  \
};                                                           \

/*! @brief Create a global buffer
 *
 * @note Increases the RAM consumption but does affect the stack size. Use \p BSX_CREATE_FIFO_LOCAL to
 *       avoid allocating global variables. */
#define BSX_CREATE_FIFO_GLOBAL(name, type, n_dims, n_frames)   \
bsx_data_content_t bsx_##name##_buffer_g[(n_dims)*(n_frames)]; \
bsx_fifo_data_t bsx_##name##_g =                               \
{                                                              \
        0,                                                     \
        (bsx_##name##_buffer_g),                               \
        (type),                                                \
        0U,                                                    \
        (n_dims),                                              \
        (n_frames)                                             \
};

/*! @brief Create many function local buffers
 *
 * @note If used within a function, the stack size may be increased tremendously depending on the
 *       dimensions and frames. Use \p BSX_CREATE_FIFO_GLOBAL to allocate globally in RAM instead
 *       of putting it on the stack.
 */
#define BSX_CREATE_FIFOS_LOCAL(name, count, type, n_dims, n_frames)       \
bsx_data_content_t bsx_##name##_buffer[(count)][(n_dims)*(n_frames)];     \
bsx_fifo_data_t bsx_##name[count];                                        \
{                                                                         \
    size_t idx;                                                           \
    size_t n_buffer = sizeof(bsx_data_content_t)*(n_dims)*(n_frames);     \
    for (idx = 0U; idx < count; ++idx)                                    \
    {                                                                     \
        bsx_##name[idx].time_stamp = 0U;                                  \
        bsx_##name[idx].content_p = (bsx_##name##_buffer)[idx];           \
        memset(bsx_##name##_buffer[idx], 0x0U, n_buffer);                 \
        bsx_##name[idx].data_type = (type);                               \
        bsx_##name[idx].sensor_id = BSX_SENSOR_ID_INVALID;                \
        bsx_##name[idx].dims = (n_dims);                                  \
        bsx_##name[idx].depth = (n_frames);                               \
    }                                                                     \
}


/*! @brief Identifiers for data types
 *
 * Allow computation of sizes during run-time for buffers hidden by pointers.
 */
typedef enum
{
    BSX_BUFFER_TYPE_FLOAT = 4, /**< single precision floating-point as defined within IEEE-754 */
    BSX_BUFFER_TYPE_S32   = 5, /**< signed 32 bit integer */
    BSX_BUFFER_TYPE_U32   = 6, /**< unsigned 32 bit integer */
    BSX_BUFFER_TYPE_S16   = 7, /**< signed 16 bit integer */
    BSX_BUFFER_TYPE_U16   = 8, /**< unsigned 16 bit integer */
    BSX_BUFFER_TYPE_S8    = 9, /**< signed 8 bit integer */
    BSX_BUFFER_TYPE_U8    = 10, /**< unsigned 8 bit integer */
    BSX_BUFFER_TYPE_UNKNOWN = 11
} bsx_buffer_datatype_identifier_t;


/*! @brief Thread identifiers for grouping of channel processing
 *
 * @{
 */
#define BSX_THREAD_MAIN           (0x01U) /**< Identifier for the main thread */
#define BSX_THREAD_CALIBRATION    (0x02U) /**< Identifier for the thread running self-calibration */
/*! @} */

#ifdef __cplusplus
}
#endif

#endif /* __BSX_DATATYPES_H__ */
/*! @}*/
