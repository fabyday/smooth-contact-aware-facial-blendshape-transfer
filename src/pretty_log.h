#pragma once
#include <eigen/Core>
#include <eigen/Sparse>
//#if  defined(DEBUG) && defined(PRINT_PRETTY_LOG)


// TRICK. Compiler will remove "if" state, because it is constant.
// https://stackoverflow.com/questions/21256252/force-the-compiler-to-ignore-some-lines-in-the-program
#if defined(PRINT_PRETTY_LOG_ALL) || defined(PRINT_PRETTY_LOG_DENSE)
#define PRETTY_PRINT_BLOCK_FLAG 1
#else 
#define PRETTY_PRINT_BLOCK_FLAG 0
#endif 
#define PRETTY_LOG_BEGIN(msg) if(PRETTY_PRINT_BLOCK_FLAG)\
								{	log_msg(("===="msg"====")) \
									log_msg("log_state_begin") 

#define PRETTY_LOG_END() log_msg("log_state_end") }



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




