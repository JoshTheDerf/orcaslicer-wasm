#ifndef ORCA_WASM_SHIMS_EXPAT_H
#define ORCA_WASM_SHIMS_EXPAT_H

#include <stddef.h>
#include <stdlib.h>
#ifdef __cplusplus
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#else
#include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef char XML_Char;
#define XMLCALL

typedef enum {
    XML_STATUS_ERROR = 0,
    XML_STATUS_OK = 1
} XML_Status;

typedef enum {
    XML_ERROR_NONE = 0,
    XML_ERROR_GENERIC = 1
} XML_Error;

typedef struct XML_ParserStruct* XML_Parser;

typedef void (*XML_StartElementHandler)(void* userData, const char* name, const char** atts);
typedef void (*XML_EndElementHandler)(void* userData, const char* name);
typedef void (*XML_CharacterDataHandler)(void* userData, const XML_Char* s, int len);

struct XML_ParserStruct {
    void* user_data;
    XML_StartElementHandler start_handler;
    XML_EndElementHandler end_handler;
    XML_CharacterDataHandler character_handler;
    XML_Error last_error;
    long current_line;
    int stopped;
};

static inline XML_Parser XML_ParserCreate(const XML_Char* /*encoding*/)
{
    struct XML_ParserStruct* parser = (struct XML_ParserStruct*)malloc(sizeof(struct XML_ParserStruct));
    if (parser != NULL) {
        parser->user_data = NULL;
        parser->start_handler = NULL;
        parser->end_handler = NULL;
        parser->character_handler = NULL;
        parser->last_error = XML_ERROR_NONE;
        parser->current_line = 0;
        parser->stopped = 0;
    }
    return parser;
}

static inline void XML_SetUserData(XML_Parser parser, void* userData)
{
    if (parser != NULL) parser->user_data = userData;
}

static inline void XML_SetElementHandler(XML_Parser parser, XML_StartElementHandler start, XML_EndElementHandler end)
{
    if (parser != NULL) {
        parser->start_handler = start;
        parser->end_handler = end;
    }
}

static inline void XML_SetCharacterDataHandler(XML_Parser parser, XML_CharacterDataHandler handler)
{
    if (parser != NULL) parser->character_handler = handler;
}

/* Orca v2.3.2's bbs_3mf.cpp installs XML_SetEntityDeclHandler(nullptr) and
 * XML_SetExternalEntityRefHandler(nullptr) defensively to disable entity
 * expansion (XXE hardening). The shim parses no input, so these can be
 * no-op stubs. Signatures match upstream expat so the calls compile. */
typedef void (*XML_EntityDeclHandler)(void* userData,
                                      const XML_Char* entityName,
                                      int is_parameter_entity,
                                      const XML_Char* value,
                                      int value_length,
                                      const XML_Char* base,
                                      const XML_Char* systemId,
                                      const XML_Char* publicId,
                                      const XML_Char* notationName);

typedef int (*XML_ExternalEntityRefHandler)(XML_Parser parser,
                                            const XML_Char* context,
                                            const XML_Char* base,
                                            const XML_Char* systemId,
                                            const XML_Char* publicId);

static inline void XML_SetEntityDeclHandler(XML_Parser /*parser*/, XML_EntityDeclHandler /*handler*/) {}
static inline void XML_SetExternalEntityRefHandler(XML_Parser /*parser*/, XML_ExternalEntityRefHandler /*handler*/) {}

static inline void* XML_GetBuffer(XML_Parser /*parser*/, int size)
{
    if (size <= 0) return NULL;
    return malloc((size_t)size);
}

static inline XML_Status XML_ParseBuffer(XML_Parser parser, int len, int /*isFinal*/)
{
    if (parser == NULL || len < 0) return XML_STATUS_ERROR;
    if (parser->character_handler != NULL && len > 0) {
        XML_Char* dummy = (XML_Char*)malloc((size_t)len);
        if (dummy == NULL) return XML_STATUS_ERROR;
        parser->character_handler(parser->user_data, dummy, len);
        free(dummy);
    }
    return XML_STATUS_OK;
}

static inline XML_Status XML_Parse(XML_Parser parser, const char* s, int len, int /*isFinal*/)
{
    if (parser == NULL || len < 0) return XML_STATUS_ERROR;
    if (parser->character_handler != NULL && s != NULL && len > 0) {
        parser->character_handler(parser->user_data, s, len);
    }
    return XML_STATUS_OK;
}

static inline unsigned XML_GetSpecifiedAttributeCount(XML_Parser /*parser*/)
{
    return 0;
}

static inline long XML_GetCurrentLineNumber(XML_Parser parser)
{
    return (parser != NULL) ? parser->current_line : 0;
}

static inline void XML_StopParser(XML_Parser parser, int /*resumable*/)
{
    if (parser != NULL) parser->stopped = 1;
}

static inline void XML_ParserFree(XML_Parser parser)
{
    if (parser != NULL) free(parser);
}

static inline XML_Error XML_GetErrorCode(XML_Parser parser)
{
    return (parser != NULL) ? parser->last_error : XML_ERROR_GENERIC;
}

static inline const XML_Char* XML_ErrorString(XML_Error /*code*/)
{
    return "expat unavailable";
}

static inline int XML_ParserReset(XML_Parser parser, const XML_Char* /*encoding*/)
{
    if (parser == NULL) return 0;
    parser->last_error = XML_ERROR_NONE;
    parser->current_line = 0;
    parser->stopped = 0;
    return 1;
}

#ifdef __cplusplus
}
#endif

#endif /* ORCA_WASM_SHIMS_EXPAT_H */
