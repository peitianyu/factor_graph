#ifndef __UTILS_H__
#define __UTILS_H__

#ifdef GRAPH_ENABLE_LOG
#define GRAPH_LOG(...) \
    printf("%s:%i: ", __func__, __LINE__); \
    printf(__VA_ARGS__); \
    printf("\n");
#else
#define GRAPH_LOG(...) ((void)0)
#endif // GRAPH_ENABLE_LOG

#ifdef GRAPH_ENABLE_ASSERT
#define GRAPH_ASSERT(expr) \
    if (!!(expr)); \
    else { \
    fprintf(stderr, "Assertion failed: %s at %s:%i (%s)\n", #expr, __FILE__, __LINE__, __func__); \
    std::abort(); }
#else
#define GRAPH_ASSERT(expr) ((void)0)
#endif // GRAPH_ENABLE_ASSERT

#endif // __UTILS_H__