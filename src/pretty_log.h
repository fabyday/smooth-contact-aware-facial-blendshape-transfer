#pragma once
#include <eigen/Core>
#include <eigen/Sparse>
#include <iostream>

//for helper macro...
//see
//https://stackoverflow.com/questions/9183993/msvc-variadic-macro-expansion
#define COUNT_ASSERT_ARGS_IMPL2(_1, _2, count, ...) \
   count
#define COUNT_ASSERT_ARGS_IMPL(args) \
   COUNT_ASSERT_ARGS_IMPL2 args
#define COUNT_ASSERT_ARGS(...) \
   COUNT_ASSERT_ARGS_IMPL((__VA_ARGS__, 2, 1, 0))
 /* Pick the right helper macro to invoke. */


#define GLUE_FUNC_NAME_HELPER2(prefix, count) prefix##count
#define GLUE_FUNC_NAME_HELPER1(prefix, count) GLUE_FUNC_NAME_HELPER2(prefix, count)
#define GLUE_FUNC_NAME_HELPER(prefix, count) GLUE_FUNC_NAME_HELPER1(prefix, count)
#define GLUE_FUNC_AND_PARAMS(x,y) x y




//////////////////////////////////////////////////////////////////////////////////////////////////////






// TRICK. Compiler will remove "if" state, because it is constant.
// https://stackoverflow.com/questions/21256252/force-the-compiler-to-ignore-some-lines-in-the-program
#if defined(PRINT_PRETTY_LOG_ALL) || defined(PRINT_PRETTY_LOG_DENSE)
#define PRETTY_PRINT_BLOCK_FLAG 1
#else 
#define PRETTY_PRINT_BLOCK_FLAG 0
#endif 

#if defined(LOG_CATEGORY)
	#define PRETTY_LOG_BEGIN_2(msg,const_category) if(std::string(const_category) == LOG_CATEGORY){\
													std::cout << const_category << " "<< const_category << std::endl;\
											 		if(PRETTY_PRINT_BLOCK_FLAG){\
														log_msg(("===="msg"====")) \
														log_msg("log_state_begin") 
	#define PRETTY_LOG_END_2(msg, const_category) log_msg("log_state_end") }}
#else
#define LOG_CATEGORY ""
#define PRETTY_LOG_BEGIN_2(msg,const_category) if(PRETTY_PRINT_BLOCK_FLAG){\
												log_msg(("===="msg"====")) \
												log_msg("log_state_begin")
#define PRETTY_LOG_END_2(msg, const_category) }
#endif()

#define PRETTY_LOG_BEGIN_1(msg) PRETTY_LOG_BEGIN_2(msg, LOG_CATEGORY"1")
#define PRETTY_LOG_END_1(msg) PRETTY_LOG_END_2(msg, LOG_CATEGORY"1")

#define PRETTY_LOG_BEGIN(...) \
  GLUE_FUNC_AND_PARAMS( GLUE_FUNC_NAME_HELPER( PRETTY_LOG_BEGIN_, COUNT_ASSERT_ARGS(__VA_ARGS__) ), (__VA_ARGS__))

#define PRETTY_LOG_END(...) \
  GLUE_FUNC_AND_PARAMS( GLUE_FUNC_NAME_HELPER( PRETTY_LOG_END_,COUNT_ASSERT_ARGS(__VA_ARGS__) ), (__VA_ARGS__))




#if  defined(PRINT_PRETTY_LOG_ALL)
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
//#define log_sparse(A, invV_adj, tdg) std::cout << "======" << std::endl; \
//							std::cout << invV_adj << std::endl;\
//						Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");\
//						std::cout << tdg.get_ind() << std::endl;\
//						std::cout << A.toDense().format(CleanFmt) << std::endl;
	
#define log_sparse(A) std::cout << A.toDense().format(CleanFmt) << std::endl;
						
#define log_dense(D) std::cout << D.format(CleanFmt) << std::endl;

#define log_msg(msg) std::cout << msg << std::endl;

#elif  defined(PRINT_PRETTY_LOG_DENSE)
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
#define log_sparse(A) 
#define log_dense(D) std::cout << D.format(CleanFmt) << std::endl;
#define log_msg(msg) std::cout << msg << std::endl;

#else
#define log_sparse(A) 
#define log_dense(D)
#define log_msg(msg) 

#endif // DEBUG




