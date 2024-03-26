from typing import Iterable


def sum_even_numbers(numbers: Iterable[int]) -> int:
    """Given an iterable of integers, return the sum of all even numbers in the iterable."""
    return sum(num for num in numbers if num % 2 == 0)


# import copy


# a = [[1], [2], [3]]
# b = copy.copy(a)

# b[1] = [5]

# print(a)
# print(b)
