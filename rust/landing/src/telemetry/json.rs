use crate::sys::*;
pub type size_t = usize;
pub type JsonType = ::core::ffi::c_uint;
pub const JSON_PRIMITIVE: JsonType = 4;
pub const JSON_STRING: JsonType = 3;
pub const JSON_ARRAY: JsonType = 2;
pub const JSON_OBJECT: JsonType = 1;
pub const JSON_UNDEFINED: JsonType = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct JsonToken {
    pub type_0: JsonType,
    pub start: ::core::ffi::c_int,
    pub end: ::core::ffi::c_int,
    pub size: ::core::ffi::c_int,
    pub parent: ::core::ffi::c_int,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct JsonDoc {
    pub text: *const ::core::ffi::c_char,
    pub tokens: *mut JsonToken,
    pub count: ::core::ffi::c_int,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Parser {
    pub pos: ::core::ffi::c_uint,
    pub next: ::core::ffi::c_uint,
    pub parent: ::core::ffi::c_int,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct JsonWriter {
    pub data: *mut ::core::ffi::c_char,
    pub length: size_t,
    pub capacity: size_t,
    pub failed: bool,
}
pub type C2RustUnnamed = ::core::ffi::c_uint;
pub const true_0: ::core::ffi::c_int = 1 as ::core::ffi::c_int;
pub const false_0: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
pub const NULL: *mut ::core::ffi::c_void = ::core::ptr::null_mut::<
    ::core::ffi::c_void,
>();
unsafe extern "C" fn alloc_token(
    mut p: *mut Parser,
    mut tokens: *mut JsonToken,
    mut cap: ::core::ffi::c_int,
) -> *mut JsonToken {
    if (*p).next as ::core::ffi::c_int >= cap {
        return ::core::ptr::null_mut::<JsonToken>();
    }
    let fresh3 = (*p).next;
    (*p).next = (*p).next.wrapping_add(1);
    let mut t: *mut JsonToken = tokens.offset(fresh3 as isize) as *mut JsonToken;
    (*t).type_0 = JSON_UNDEFINED;
    (*t).start = -(1 as ::core::ffi::c_int);
    (*t).end = -(1 as ::core::ffi::c_int);
    (*t).size = 0 as ::core::ffi::c_int;
    (*t).parent = -(1 as ::core::ffi::c_int);
    return t;
}
unsafe extern "C" fn fill_token(
    mut t: *mut JsonToken,
    mut type_0: JsonType,
    mut start: ::core::ffi::c_int,
    mut end: ::core::ffi::c_int,
) {
    (*t).type_0 = type_0;
    (*t).start = start;
    (*t).end = end;
    (*t).size = 0 as ::core::ffi::c_int;
}
unsafe extern "C" fn parse_primitive(
    mut p: *mut Parser,
    mut js: *const ::core::ffi::c_char,
    mut len: size_t,
    mut tokens: *mut JsonToken,
    mut cap: ::core::ffi::c_int,
) -> ::core::ffi::c_int {
    let mut start: ::core::ffi::c_uint = (*p).pos;
    while ((*p).pos as size_t) < len {
        let mut c: ::core::ffi::c_char = *js.offset((*p).pos as isize);
        if c as ::core::ffi::c_int == '\t' as i32
            || c as ::core::ffi::c_int == '\r' as i32
            || c as ::core::ffi::c_int == '\n' as i32
            || c as ::core::ffi::c_int == ' ' as i32
            || c as ::core::ffi::c_int == ',' as i32
            || c as ::core::ffi::c_int == ']' as i32
            || c as ::core::ffi::c_int == '}' as i32
        {
            break;
        }
        if (c as ::core::ffi::c_uchar as ::core::ffi::c_int) < 32 as ::core::ffi::c_int
            || c as ::core::ffi::c_int == ':' as i32
            || c as ::core::ffi::c_int == '"' as i32
        {
            return -(2 as ::core::ffi::c_int);
        }
        (*p).pos = (*p).pos.wrapping_add(1);
    }
    if !tokens.is_null() {
        let mut t: *mut JsonToken = alloc_token(p, tokens, cap);
        if t.is_null() {
            (*p).pos = start;
            return -(1 as ::core::ffi::c_int);
        }
        fill_token(
            t,
            JSON_PRIMITIVE,
            start as ::core::ffi::c_int,
            (*p).pos as ::core::ffi::c_int,
        );
        (*t).parent = (*p).parent;
    }
    (*p).pos = (*p).pos.wrapping_sub(1);
    return 0 as ::core::ffi::c_int;
}
unsafe extern "C" fn parse_string(
    mut p: *mut Parser,
    mut js: *const ::core::ffi::c_char,
    mut len: size_t,
    mut tokens: *mut JsonToken,
    mut cap: ::core::ffi::c_int,
) -> ::core::ffi::c_int {
    let fresh4 = (*p).pos;
    (*p).pos = (*p).pos.wrapping_add(1);
    let mut start: ::core::ffi::c_uint = fresh4;
    while ((*p).pos as size_t) < len {
        let mut c: ::core::ffi::c_char = *js.offset((*p).pos as isize);
        if c as ::core::ffi::c_int == '"' as i32 {
            if !tokens.is_null() {
                let mut t: *mut JsonToken = alloc_token(p, tokens, cap);
                if t.is_null() {
                    (*p).pos = start;
                    return -(1 as ::core::ffi::c_int);
                }
                fill_token(
                    t,
                    JSON_STRING,
                    start as ::core::ffi::c_int + 1 as ::core::ffi::c_int,
                    (*p).pos as ::core::ffi::c_int,
                );
                (*t).parent = (*p).parent;
            }
            return 0 as ::core::ffi::c_int;
        }
        if c as ::core::ffi::c_int == '\\' as i32
            && ((*p).pos.wrapping_add(1 as ::core::ffi::c_uint) as size_t) < len
        {
            (*p).pos = (*p).pos.wrapping_add(1);
            let mut e: ::core::ffi::c_char = *js.offset((*p).pos as isize);
            if e as ::core::ffi::c_int == 'u' as i32 {
                let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
                while i < 4 as ::core::ffi::c_int {
                    (*p).pos = (*p).pos.wrapping_add(1);
                    if (*p).pos as size_t >= len
                        || *(*__ctype_b_loc())
                            .offset(
                                *js.offset((*p).pos as isize) as ::core::ffi::c_uchar
                                    as ::core::ffi::c_int as isize,
                            ) as ::core::ffi::c_int
                            & _ISxdigit as ::core::ffi::c_int as ::core::ffi::c_ushort
                                as ::core::ffi::c_int == 0
                    {
                        return -(2 as ::core::ffi::c_int);
                    }
                    i += 1;
                }
            } else if strchr(
                    b"\"/bfnrt\0" as *const u8 as *const ::core::ffi::c_char,
                    e as ::core::ffi::c_int,
                )
                .is_null()
            {
                return -(2 as ::core::ffi::c_int)
            }
        }
        (*p).pos = (*p).pos.wrapping_add(1);
    }
    (*p).pos = start;
    return -(3 as ::core::ffi::c_int);
}
#[no_mangle]
pub unsafe extern "C" fn json_parse(
    mut text: *const ::core::ffi::c_char,
    mut tokens: *mut JsonToken,
    mut capacity: ::core::ffi::c_int,
    mut doc: *mut JsonDoc,
) -> ::core::ffi::c_int {
    let mut p: Parser = Parser {
        pos: 0 as ::core::ffi::c_uint,
        next: 0 as ::core::ffi::c_uint,
        parent: -(1 as ::core::ffi::c_int),
    };
    let mut len: size_t = strlen(text);
    while (p.pos as size_t) < len {
        let mut c: ::core::ffi::c_char = *text.offset(p.pos as isize);
        match c as ::core::ffi::c_int {
            123 | 91 => {
                let mut t: *mut JsonToken = alloc_token(&raw mut p, tokens, capacity);
                if t.is_null() {
                    return -(1 as ::core::ffi::c_int);
                }
                if p.parent != -(1 as ::core::ffi::c_int) {
                    let ref mut fresh0 = (*tokens.offset(p.parent as isize)).size;
                    *fresh0 += 1;
                }
                (*t).type_0 = (if c as ::core::ffi::c_int == '{' as i32 {
                    JSON_OBJECT as ::core::ffi::c_int
                } else {
                    JSON_ARRAY as ::core::ffi::c_int
                }) as JsonType;
                (*t).start = p.pos as ::core::ffi::c_int;
                (*t).parent = p.parent;
                p.parent = p.next as ::core::ffi::c_int - 1 as ::core::ffi::c_int;
            }
            125 | 93 => {
                let mut type_0: JsonType = (if c as ::core::ffi::c_int == '}' as i32 {
                    JSON_OBJECT as ::core::ffi::c_int
                } else {
                    JSON_ARRAY as ::core::ffi::c_int
                }) as JsonType;
                let mut i: ::core::ffi::c_int = p.next as ::core::ffi::c_int
                    - 1 as ::core::ffi::c_int;
                while i >= 0 as ::core::ffi::c_int {
                    let mut t_0: *mut JsonToken = tokens.offset(i as isize)
                        as *mut JsonToken;
                    if (*t_0).start != -(1 as ::core::ffi::c_int)
                        && (*t_0).end == -(1 as ::core::ffi::c_int)
                    {
                        if (*t_0).type_0 as ::core::ffi::c_uint
                            != type_0 as ::core::ffi::c_uint
                        {
                            return -(2 as ::core::ffi::c_int);
                        }
                        (*t_0).end = p.pos as ::core::ffi::c_int
                            + 1 as ::core::ffi::c_int;
                        p.parent = (*t_0).parent;
                        break;
                    } else {
                        i -= 1;
                    }
                }
                if i < 0 as ::core::ffi::c_int {
                    return -(2 as ::core::ffi::c_int);
                }
            }
            34 => {
                let mut r: ::core::ffi::c_int = parse_string(
                    &raw mut p,
                    text,
                    len,
                    tokens,
                    capacity,
                );
                if r < 0 as ::core::ffi::c_int {
                    return r;
                }
                if p.parent != -(1 as ::core::ffi::c_int) {
                    let ref mut fresh1 = (*tokens.offset(p.parent as isize)).size;
                    *fresh1 += 1;
                }
            }
            9 | 13 | 10 | 32 | 58 | 44 => {}
            _ => {
                let mut r_0: ::core::ffi::c_int = parse_primitive(
                    &raw mut p,
                    text,
                    len,
                    tokens,
                    capacity,
                );
                if r_0 < 0 as ::core::ffi::c_int {
                    return r_0;
                }
                if p.parent != -(1 as ::core::ffi::c_int) {
                    let ref mut fresh2 = (*tokens.offset(p.parent as isize)).size;
                    *fresh2 += 1;
                }
            }
        }
        p.pos = p.pos.wrapping_add(1);
    }
    let mut i_0: ::core::ffi::c_uint = 0 as ::core::ffi::c_uint;
    while i_0 < p.next {
        if (*tokens.offset(i_0 as isize)).start != -(1 as ::core::ffi::c_int)
            && (*tokens.offset(i_0 as isize)).end == -(1 as ::core::ffi::c_int)
        {
            return -(3 as ::core::ffi::c_int);
        }
        i_0 = i_0.wrapping_add(1);
    }
    (*doc).text = text;
    (*doc).tokens = tokens;
    (*doc).count = p.next as ::core::ffi::c_int;
    return (*doc).count;
}
#[no_mangle]
pub unsafe extern "C" fn json_skip(
    mut doc: *const JsonDoc,
    mut index: ::core::ffi::c_int,
) -> ::core::ffi::c_int {
    if index < 0 as ::core::ffi::c_int || index >= (*doc).count {
        return index + 1 as ::core::ffi::c_int;
    }
    let mut end: ::core::ffi::c_int = (*(*doc).tokens.offset(index as isize)).end;
    let mut i: ::core::ffi::c_int = index + 1 as ::core::ffi::c_int;
    while i < (*doc).count && (*(*doc).tokens.offset(i as isize)).start < end {
        i += 1;
    }
    return i;
}
#[no_mangle]
pub unsafe extern "C" fn json_token_eq(
    mut doc: *const JsonDoc,
    mut index: ::core::ffi::c_int,
    mut value: *const ::core::ffi::c_char,
) -> bool {
    if index < 0 as ::core::ffi::c_int || index >= (*doc).count {
        return false_0 != 0;
    }
    let mut t: *const JsonToken = (*doc).tokens.offset(index as isize) as *mut JsonToken;
    let mut n: size_t = strlen(value);
    return ((*t).end - (*t).start) as size_t == n
        && strncmp((*doc).text.offset((*t).start as isize), value, n)
            == 0 as ::core::ffi::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn json_object_get(
    mut doc: *const JsonDoc,
    mut object_index: ::core::ffi::c_int,
    mut key: *const ::core::ffi::c_char,
) -> ::core::ffi::c_int {
    if object_index < 0 as ::core::ffi::c_int || object_index >= (*doc).count
        || (*(*doc).tokens.offset(object_index as isize)).type_0 as ::core::ffi::c_uint
            != JSON_OBJECT as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return -(1 as ::core::ffi::c_int);
    }
    let mut object_end: ::core::ffi::c_int = (*(*doc)
        .tokens
        .offset(object_index as isize))
        .end;
    let mut i: ::core::ffi::c_int = object_index + 1 as ::core::ffi::c_int;
    while i < (*doc).count && (*(*doc).tokens.offset(i as isize)).start < object_end {
        let mut key_index: ::core::ffi::c_int = i;
        let mut value_index: ::core::ffi::c_int = i + 1 as ::core::ffi::c_int;
        if value_index >= (*doc).count {
            return -(1 as ::core::ffi::c_int);
        }
        if (*(*doc).tokens.offset(key_index as isize)).type_0 as ::core::ffi::c_uint
            == JSON_STRING as ::core::ffi::c_int as ::core::ffi::c_uint
            && json_token_eq(doc, key_index, key) as ::core::ffi::c_int != 0
        {
            return value_index;
        }
        i = json_skip(doc, value_index);
    }
    return -(1 as ::core::ffi::c_int);
}
#[no_mangle]
pub unsafe extern "C" fn json_array_get(
    mut doc: *const JsonDoc,
    mut array_index: ::core::ffi::c_int,
    mut element_index: ::core::ffi::c_int,
) -> ::core::ffi::c_int {
    if array_index < 0 as ::core::ffi::c_int || array_index >= (*doc).count
        || (*(*doc).tokens.offset(array_index as isize)).type_0 as ::core::ffi::c_uint
            != JSON_ARRAY as ::core::ffi::c_int as ::core::ffi::c_uint
    {
        return -(1 as ::core::ffi::c_int);
    }
    let mut end: ::core::ffi::c_int = (*(*doc).tokens.offset(array_index as isize)).end;
    let mut i: ::core::ffi::c_int = array_index + 1 as ::core::ffi::c_int;
    let mut n: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < (*doc).count && (*(*doc).tokens.offset(i as isize)).start < end {
        if n == element_index {
            return i;
        }
        n += 1;
        i = json_skip(doc, i);
    }
    return -(1 as ::core::ffi::c_int);
}
unsafe extern "C" fn token_copy(
    mut doc: *const JsonDoc,
    mut index: ::core::ffi::c_int,
    mut buf: *mut ::core::ffi::c_char,
    mut cap: size_t,
) -> bool {
    if index < 0 as ::core::ffi::c_int || index >= (*doc).count || cap == 0 as size_t {
        return false_0 != 0;
    }
    let mut n: ::core::ffi::c_int = (*(*doc).tokens.offset(index as isize)).end
        - (*(*doc).tokens.offset(index as isize)).start;
    if n < 0 as ::core::ffi::c_int {
        return false_0 != 0;
    }
    let mut copy: size_t = if (n as size_t) < cap.wrapping_sub(1 as size_t) {
        n as size_t
    } else {
        cap.wrapping_sub(1 as size_t)
    };
    memcpy(
        buf as *mut ::core::ffi::c_void,
        (*doc).text.offset((*(*doc).tokens.offset(index as isize)).start as isize)
            as *const ::core::ffi::c_void,
        copy,
    );
    *buf.offset(copy as isize) = 0 as ::core::ffi::c_char;
    return true_0 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn json_number(
    mut doc: *const JsonDoc,
    mut index: ::core::ffi::c_int,
    mut fallback: ::core::ffi::c_double,
) -> ::core::ffi::c_double {
    let mut buf: [::core::ffi::c_char; 96] = [0; 96];
    if !token_copy(
        doc,
        index,
        &raw mut buf as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 96]>() as size_t,
    ) {
        return fallback;
    }
    *__errno_location() = 0 as ::core::ffi::c_int;
    let mut end: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
        ::core::ffi::c_char,
    >();
    let mut v: ::core::ffi::c_double = strtod(
        &raw mut buf as *mut ::core::ffi::c_char,
        &raw mut end,
    );
    return if *__errno_location() == 0 as ::core::ffi::c_int
        && end != &raw mut buf as *mut ::core::ffi::c_char && v.is_finite() as i32 != 0
    {
        v
    } else {
        fallback
    };
}
#[no_mangle]
pub unsafe extern "C" fn json_integer(
    mut doc: *const JsonDoc,
    mut index: ::core::ffi::c_int,
    mut fallback: ::core::ffi::c_long,
) -> ::core::ffi::c_long {
    let mut buf: [::core::ffi::c_char; 64] = [0; 64];
    if !token_copy(
        doc,
        index,
        &raw mut buf as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
    ) {
        return fallback;
    }
    *__errno_location() = 0 as ::core::ffi::c_int;
    let mut end: *mut ::core::ffi::c_char = ::core::ptr::null_mut::<
        ::core::ffi::c_char,
    >();
    let mut v: ::core::ffi::c_long = strtol(
        &raw mut buf as *mut ::core::ffi::c_char,
        &raw mut end,
        10 as ::core::ffi::c_int,
    );
    return if *__errno_location() == 0 as ::core::ffi::c_int
        && end != &raw mut buf as *mut ::core::ffi::c_char
    {
        v
    } else {
        fallback
    };
}
#[no_mangle]
pub unsafe extern "C" fn json_boolean(
    mut doc: *const JsonDoc,
    mut index: ::core::ffi::c_int,
    mut fallback: bool,
) -> bool {
    if json_token_eq(doc, index, b"true\0" as *const u8 as *const ::core::ffi::c_char) {
        return true_0 != 0;
    }
    if json_token_eq(doc, index, b"false\0" as *const u8 as *const ::core::ffi::c_char) {
        return false_0 != 0;
    }
    return fallback;
}
#[no_mangle]
pub unsafe extern "C" fn json_string(
    mut doc: *const JsonDoc,
    mut index: ::core::ffi::c_int,
    mut out: *mut ::core::ffi::c_char,
    mut out_size: size_t,
) -> bool {
    if index < 0 as ::core::ffi::c_int || index >= (*doc).count
        || (*(*doc).tokens.offset(index as isize)).type_0 as ::core::ffi::c_uint
            != JSON_STRING as ::core::ffi::c_int as ::core::ffi::c_uint
        || out_size == 0 as size_t
    {
        return false_0 != 0;
    }
    let mut src: *const ::core::ffi::c_char = (*doc)
        .text
        .offset((*(*doc).tokens.offset(index as isize)).start as isize);
    let mut len: ::core::ffi::c_int = (*(*doc).tokens.offset(index as isize)).end
        - (*(*doc).tokens.offset(index as isize)).start;
    let mut o: size_t = 0 as size_t;
    let mut i: ::core::ffi::c_int = 0 as ::core::ffi::c_int;
    while i < len && o.wrapping_add(1 as size_t) < out_size {
        let mut c: ::core::ffi::c_char = *src.offset(i as isize);
        if c as ::core::ffi::c_int == '\\' as i32 && (i + 1 as ::core::ffi::c_int) < len
        {
            i += 1;
            let mut e: ::core::ffi::c_char = *src.offset(i as isize);
            match e as ::core::ffi::c_int {
                110 => {
                    c = '\n' as i32 as ::core::ffi::c_char;
                }
                114 => {
                    c = '\r' as i32 as ::core::ffi::c_char;
                }
                116 => {
                    c = '\t' as i32 as ::core::ffi::c_char;
                }
                98 => {
                    c = '\u{8}' as i32 as ::core::ffi::c_char;
                }
                102 => {
                    c = '\u{c}' as i32 as ::core::ffi::c_char;
                }
                34 => {
                    c = '"' as i32 as ::core::ffi::c_char;
                }
                92 => {
                    c = '\\' as i32 as ::core::ffi::c_char;
                }
                47 => {
                    c = '/' as i32 as ::core::ffi::c_char;
                }
                _ => {
                    c = '?' as i32 as ::core::ffi::c_char;
                }
            }
        }
        let fresh5 = o;
        o = o.wrapping_add(1);
        *out.offset(fresh5 as isize) = c;
        i += 1;
    }
    *out.offset(o as isize) = 0 as ::core::ffi::c_char;
    return true_0 != 0;
}
unsafe extern "C" fn jw_reserve(mut w: *mut JsonWriter, mut extra: size_t) {
    if (*w).failed {
        return;
    }
    let mut need: size_t = (*w).length.wrapping_add(extra).wrapping_add(1 as size_t);
    if need <= (*w).capacity {
        return;
    }
    let mut cap: size_t = if (*w).capacity != 0 {
        (*w).capacity
    } else {
        1024 as size_t
    };
    while cap < need {
        cap = cap.wrapping_mul(2 as size_t);
    }
    let mut next: *mut ::core::ffi::c_char = realloc(
        (*w).data as *mut ::core::ffi::c_void,
        cap,
    ) as *mut ::core::ffi::c_char;
    if next.is_null() {
        (*w).failed = true_0 != 0;
        return;
    }
    (*w).data = next;
    (*w).capacity = cap;
}
#[no_mangle]
pub unsafe extern "C" fn jw_init(mut w: *mut JsonWriter) {
    memset(
        w as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<JsonWriter>() as size_t,
    );
    jw_reserve(w, 0 as size_t);
    if !(*w).data.is_null() {
        *(*w).data.offset(0 as ::core::ffi::c_int as isize) = 0 as ::core::ffi::c_char;
    }
}
#[no_mangle]
pub unsafe extern "C" fn jw_free(mut w: *mut JsonWriter) {
    free((*w).data as *mut ::core::ffi::c_void);
    memset(
        w as *mut ::core::ffi::c_void,
        0 as ::core::ffi::c_int,
        ::core::mem::size_of::<JsonWriter>() as size_t,
    );
}
#[no_mangle]
pub unsafe extern "C" fn jw_raw(
    mut w: *mut JsonWriter,
    mut text: *const ::core::ffi::c_char,
) {
    let mut n: size_t = strlen(text);
    jw_reserve(w, n);
    if (*w).failed {
        return;
    }
    memcpy(
        (*w).data.offset((*w).length as isize) as *mut ::core::ffi::c_void,
        text as *const ::core::ffi::c_void,
        n.wrapping_add(1 as size_t),
    );
    (*w).length = (*w).length.wrapping_add(n);
}
#[no_mangle]
pub unsafe extern "C" fn jw_char(mut w: *mut JsonWriter, mut c: ::core::ffi::c_char) {
    jw_reserve(w, 1 as size_t);
    if (*w).failed {
        return;
    }
    let fresh6 = (*w).length;
    (*w).length = (*w).length.wrapping_add(1);
    *(*w).data.offset(fresh6 as isize) = c;
    *(*w).data.offset((*w).length as isize) = 0 as ::core::ffi::c_char;
}
#[no_mangle]
pub unsafe extern "C" fn jw_string(
    mut w: *mut JsonWriter,
    mut text: *const ::core::ffi::c_char,
) {
    jw_char(w, '"' as i32 as ::core::ffi::c_char);
    if text.is_null() {
        text = b"\0" as *const u8 as *const ::core::ffi::c_char;
    }
    let mut p: *const ::core::ffi::c_uchar = text as *const ::core::ffi::c_uchar;
    while *p != 0 {
        match *p as ::core::ffi::c_int {
            34 => {
                jw_raw(w, b"\\\"\0" as *const u8 as *const ::core::ffi::c_char);
            }
            92 => {
                jw_raw(w, b"\\\\\0" as *const u8 as *const ::core::ffi::c_char);
            }
            8 => {
                jw_raw(w, b"\\b\0" as *const u8 as *const ::core::ffi::c_char);
            }
            12 => {
                jw_raw(w, b"\\f\0" as *const u8 as *const ::core::ffi::c_char);
            }
            10 => {
                jw_raw(w, b"\\n\0" as *const u8 as *const ::core::ffi::c_char);
            }
            13 => {
                jw_raw(w, b"\\r\0" as *const u8 as *const ::core::ffi::c_char);
            }
            9 => {
                jw_raw(w, b"\\t\0" as *const u8 as *const ::core::ffi::c_char);
            }
            _ => {
                if (*p as ::core::ffi::c_int) < 32 as ::core::ffi::c_int {
                    let mut buf: [::core::ffi::c_char; 8] = [0; 8];
                    snprintf(
                        &raw mut buf as *mut ::core::ffi::c_char,
                        ::core::mem::size_of::<[::core::ffi::c_char; 8]>() as size_t,
                        b"\\u%04x\0" as *const u8 as *const ::core::ffi::c_char,
                        *p as ::core::ffi::c_int,
                    );
                    jw_raw(w, &raw mut buf as *mut ::core::ffi::c_char);
                } else {
                    jw_char(w, *p as ::core::ffi::c_char);
                }
            }
        }
        p = p.offset(1);
    }
    jw_char(w, '"' as i32 as ::core::ffi::c_char);
}
#[no_mangle]
pub unsafe extern "C" fn jw_number(
    mut w: *mut JsonWriter,
    mut value: ::core::ffi::c_double,
) {
    if value.is_finite() as i32 == 0 {
        jw_raw(w, b"0\0" as *const u8 as *const ::core::ffi::c_char);
        return;
    }
    let mut buf: [::core::ffi::c_char; 64] = [0; 64];
    snprintf(
        &raw mut buf as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 64]>() as size_t,
        b"%.15g\0" as *const u8 as *const ::core::ffi::c_char,
        value,
    );
    jw_raw(w, &raw mut buf as *mut ::core::ffi::c_char);
}
#[no_mangle]
pub unsafe extern "C" fn jw_integer(
    mut w: *mut JsonWriter,
    mut value: ::core::ffi::c_longlong,
) {
    let mut buf: [::core::ffi::c_char; 48] = [0; 48];
    snprintf(
        &raw mut buf as *mut ::core::ffi::c_char,
        ::core::mem::size_of::<[::core::ffi::c_char; 48]>() as size_t,
        b"%lld\0" as *const u8 as *const ::core::ffi::c_char,
        value,
    );
    jw_raw(w, &raw mut buf as *mut ::core::ffi::c_char);
}
#[no_mangle]
pub unsafe extern "C" fn jw_bool(mut w: *mut JsonWriter, mut value: bool) {
    jw_raw(
        w,
        if value as ::core::ffi::c_int != 0 {
            b"true\0" as *const u8 as *const ::core::ffi::c_char
        } else {
            b"false\0" as *const u8 as *const ::core::ffi::c_char
        },
    );
}
#[no_mangle]
pub unsafe extern "C" fn jw_null(mut w: *mut JsonWriter) {
    jw_raw(w, b"null\0" as *const u8 as *const ::core::ffi::c_char);
}
