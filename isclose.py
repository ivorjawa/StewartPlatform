#https://github.com/scikit-hep/scikit-hep/blob/master/skhep/math/isclose.py
import umath as cmath # I'm not using complex numbers, I hope.

def isclose(a, b, rel_tol=1e-9, abs_tol=0.0, method="weak"):
    """
    returns True if a is close in value to b. False otherwise

    :param a: one of the values to be tested

    :param b: the other value to be tested

    :param rel_tol=1e-8: The relative tolerance -- the amount of error
                         allowed, relative to the magnitude of the input
                         values.

    :param abs_tol=0.0: The minimum absolute tolerance level -- useful for
                        comparisons to zero.

    :param method: The method to use. options are:
                  "asymmetric" : the b value is used for scaling the tolerance
                  "strong" : The tolerance is scaled by the smaller of
                             the two values
                  "weak" : The tolerance is scaled by the larger of
                           the two values
                  "average" : The tolerance is scaled by the average of
                              the two values.

    NOTES:

    -inf, inf and NaN behave similar to the IEEE 754 standard. That
    -is, NaN is not close to anything, even itself. inf and -inf are
    -only close to themselves.

    Complex values are compared based on their absolute value.

    The function can be used with Decimal types, if the tolerance(s) are
    specified as Decimals::

      isclose(a, b, rel_tol=Decimal('1e-9'))

    See PEP-0485 for a detailed description

    """
    if method not in ("asymmetric", "strong", "weak", "average"):
        raise ValueError(
            'method must be one of: "asymmetric",' ' "strong", "weak", "average"'
        )

    if rel_tol < 0.0 or abs_tol < 0.0:
        raise ValueError("error tolerances must be non-negative")

    if a == b:  # short-circuit exact equality
        return True
    # use cmath so it will work with complex or float
    # FIXME deliberately broken in pybricks
    if cmath.isinf(a) or cmath.isinf(b):
        # This includes the case of two infinities of opposite sign, or
        # one infinity and one finite number. Two infinities of opposite sign
        # would otherwise have an infinite relative tolerance.
        return False
    diff = abs(b - a)
    if method == "asymmetric":
        return (diff <= abs(rel_tol * b)) or (diff <= abs_tol)
    elif method == "strong":
        return ((diff <= abs(rel_tol * b)) and (diff <= abs(rel_tol * a))) or (
            diff <= abs_tol
        )
    elif method == "weak":
        return ((diff <= abs(rel_tol * b)) or (diff <= abs(rel_tol * a))) or (
            diff <= abs_tol
        )
    elif method == "average":
        return diff <= abs(rel_tol * (a + b) / 2) or (diff <= abs_tol)
    else:
        raise ValueError(
            "method must be one of:" ' "asymmetric", "strong", "weak", "average"'
        )
