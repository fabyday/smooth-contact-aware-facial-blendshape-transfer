#include "Primitives.h"

#include <iostream>
int main() {
	int s = 0;

	_8BIT_FLAG test;
	test = BIT(2);
	BIT_ON(test, BIT(3));
	if(BIT_CHECK(test, BIT(2)))
		std::cout << "test 2bit on" << std::endl;

	if (BIT_CHECK(test, BIT(3)))
		std::cout << "test 3bit on" << std::endl;
	
		_8BIT_FLAG query = 0;
		BIT_ON(query, BIT(2));
		BIT_ON(query, BIT(3));
	if (BIT_CHECK(test, query))
		std::cout << "test 2 and 3 bit on" << std::endl;
	
	if (BIT_CHECK(test, BIT(7)))
		std::cout << "test 7 bit on" << std::endl;

	if (BIT_CHECK(test, BIT(6)))
		std::cout << "test 7 bit on" << std::endl;


	_8BIT_FLAG query2 = 0;
	BIT_ON(query2, BIT(6));
	BIT_ON(query2, BIT(7));
	if (BIT_CHECK(test, query2))
		std::cout << "test 7 6 bit on" << std::endl;

	_8BIT_FLAG query3 = 0;
	BIT_ON(query3, BIT(6));
	BIT_ON(query3, BIT(2));
	if (BIT_CHECK(test, query3))
		std::cout << "test 7 6 bit on" << std::endl;



}