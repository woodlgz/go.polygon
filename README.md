
# go.polygon

This is a light-weight polygon algorithm go package which provides irregular polygon testing(includes polygon-point relation testing, polygon-line relation testing, polygon-polygon relation testing).

Note that polygon-point relation testing is specially optimized by grid trick (giving O(1) testing complexity regardless of the shape of polygon), which is useful when a polygon ,especially with complicated structure, never changes its shape.

Also note that the point axis system is a 2d planar system where x goes rightwards, y goes downwards.

This project is not licensed, so feel free to use it in your commercial project. Any contribution is welcome.