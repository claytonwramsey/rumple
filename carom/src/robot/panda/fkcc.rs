use core::simd::{LaneCount, Simd, SupportedLaneCount};

use std::simd::StdFloat;

use crate::{
    env::World3d,
    robot::{sphere_environment_in_collision, sphere_sphere_self_collision},
};

#[expect(
    non_snake_case,
    clippy::too_many_lines,
    clippy::cognitive_complexity,
    clippy::unreadable_literal,
    clippy::approx_constant
)]
pub fn interleaved_sphere_fk<const L: usize>(
    q: &super::ConfigurationBlock<L>,
    environment: &World3d<f32>,
) -> bool
where
    LaneCount<L>: SupportedLaneCount,
{
    if
    /* panda_link0 */
    sphere_environment_in_collision(
        environment,
        Simd::splat(0.0f32),
        Simd::splat(0.0f32),
        Simd::splat(0.05f32),
        Simd::splat(0.08f32),
    ) {
        return false;
    } // (0, 0)
    let INPUT_0 = q[0];
    let DIV_8 = INPUT_0 * Simd::splat(0.5f32);
    let SIN_9 = DIV_8.sin();
    let COS_15 = DIV_8.cos();
    let MUL_1575 = COS_15 * SIN_9;
    let MUL_1574 = SIN_9 * SIN_9;
    let MUL_1594 = MUL_1575 * Simd::splat(2.0f32);
    let MUL_1620 = MUL_1594 * Simd::splat(0.0265023f32);
    let MUL_1584 = MUL_1574 * Simd::splat(2.0f32);
    let SUB_1587 = Simd::splat(1.0f32) - MUL_1584;
    let MUL_1613 = SUB_1587 * Simd::splat(4.21e-05f32);
    let ADD_1636 = MUL_1613 + MUL_1620;
    let MUL_1623 = SUB_1587 * Simd::splat(0.0265023f32);
    let MUL_1615 = MUL_1594 * Simd::splat(4.21e-05f32);
    let SUB_1637 = MUL_1615 - MUL_1623;
    let MUL_1650 = MUL_1594 * Simd::splat(0.08f32);
    let MUL_1653 = SUB_1587 * Simd::splat(0.08f32);
    let NEGATE_1654 = -MUL_1653;
    let MUL_1674 = MUL_1594 * Simd::splat(0.03f32);
    let MUL_1677 = SUB_1587 * Simd::splat(0.03f32);
    let NEGATE_1678 = -MUL_1677;
    if
    /* panda_link1 */
    sphere_environment_in_collision(
        environment,
        ADD_1636,
        SUB_1637,
        Simd::splat(0.2598976f32),
        Simd::splat(0.144259f32),
    ) {
        if sphere_environment_in_collision(
            environment,
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
        ) {
            return false;
        }
    } // (0, 21)
    let MUL_74 = COS_15 * Simd::splat(0.7071068f32);
    let MUL_72 = SIN_9 * Simd::splat(0.7071068f32);
    let INPUT_1 = q[1];
    let DIV_117 = INPUT_1 * Simd::splat(0.5f32);
    let SIN_118 = DIV_117.sin();
    let COS_124 = DIV_117.cos();
    let MUL_143 = MUL_72 * COS_124;
    let MUL_128 = MUL_72 * SIN_118;
    let MUL_126 = MUL_74 * COS_124;
    let SUB_149 = MUL_126 - MUL_128;
    let ADD_130 = MUL_126 + MUL_128;
    let MUL_140 = MUL_74 * SIN_118;
    let SUB_138 = MUL_140 - MUL_143;
    let ADD_144 = MUL_140 + MUL_143;
    let MUL_1757 = ADD_130 * ADD_144;
    let MUL_1755 = ADD_130 * SUB_138;
    let MUL_1750 = SUB_149 * ADD_144;
    let ADD_1775 = MUL_1755 + MUL_1750;
    let MUL_1778 = ADD_1775 * Simd::splat(2.0f32);
    let MUL_1816 = MUL_1778 * Simd::splat(0.074083f32);
    let MUL_1751 = SUB_149 * SUB_138;
    let SUB_1790 = MUL_1751 - MUL_1757;
    let MUL_1792 = SUB_1790 * Simd::splat(2.0f32);
    let MUL_1826 = MUL_1792 * Simd::splat(0.0265382f32);
    let MUL_1749 = ADD_144 * ADD_144;
    let MUL_1748 = SUB_138 * SUB_138;
    let ADD_1760 = MUL_1748 + MUL_1749;
    let MUL_1763 = ADD_1760 * Simd::splat(2.0f32);
    let SUB_1766 = Simd::splat(1.0f32) - MUL_1763;
    let MUL_1805 = SUB_1766 * Simd::splat(2.72e-05f32);
    let SUB_1831 = MUL_1816 - MUL_1805;
    let ADD_1835 = SUB_1831 + MUL_1826;
    let SUB_1767 = MUL_1750 - MUL_1755;
    let MUL_1752 = ADD_130 * ADD_130;
    let ADD_1780 = MUL_1749 + MUL_1752;
    let MUL_1783 = ADD_1780 * Simd::splat(2.0f32);
    let SUB_1786 = Simd::splat(1.0f32) - MUL_1783;
    let MUL_1819 = SUB_1786 * Simd::splat(0.074083f32);
    let MUL_1769 = SUB_1767 * Simd::splat(2.0f32);
    let MUL_1809 = MUL_1769 * Simd::splat(2.72e-05f32);
    let ADD_1832 = MUL_1809 + MUL_1819;
    let MUL_1753 = SUB_149 * ADD_130;
    let MUL_1759 = SUB_138 * ADD_144;
    let ADD_1793 = MUL_1759 + MUL_1753;
    let MUL_1795 = ADD_1793 * Simd::splat(2.0f32);
    let MUL_1828 = MUL_1795 * Simd::splat(0.0265382f32);
    let SUB_1836 = MUL_1828 - ADD_1832;
    let SUB_1787 = MUL_1759 - MUL_1753;
    let ADD_1770 = MUL_1757 + MUL_1751;
    let ADD_1796 = MUL_1748 + MUL_1752;
    let MUL_1799 = ADD_1796 * Simd::splat(2.0f32);
    let SUB_1802 = Simd::splat(1.0f32) - MUL_1799;
    let MUL_1830 = SUB_1802 * Simd::splat(0.0265382f32);
    let MUL_1789 = SUB_1787 * Simd::splat(2.0f32);
    let MUL_1823 = MUL_1789 * Simd::splat(0.074083f32);
    let MUL_1773 = ADD_1770 * Simd::splat(2.0f32);
    let MUL_1813 = MUL_1773 * Simd::splat(2.72e-05f32);
    let SUB_1834 = MUL_1813 - MUL_1823;
    let ADD_1837 = SUB_1834 + MUL_1830;
    let ADD_1838 = Simd::splat(0.333f32) + ADD_1837;
    let MUL_1852 = MUL_1792 * Simd::splat(0.03f32);
    let MUL_1854 = MUL_1795 * Simd::splat(0.03f32);
    let MUL_1856 = SUB_1802 * Simd::splat(0.03f32);
    let ADD_1857 = Simd::splat(0.333f32) + MUL_1856;
    let MUL_1871 = MUL_1792 * Simd::splat(0.08f32);
    let MUL_1873 = MUL_1795 * Simd::splat(0.08f32);
    let MUL_1875 = SUB_1802 * Simd::splat(0.08f32);
    let ADD_1876 = Simd::splat(0.333f32) + MUL_1875;
    let MUL_1885 = MUL_1778 * Simd::splat(0.12f32);
    let MUL_1888 = SUB_1786 * Simd::splat(0.12f32);
    let NEGATE_1889 = -MUL_1888;
    let MUL_1892 = MUL_1789 * Simd::splat(0.12f32);
    let SUB_1900 = Simd::splat(0.333f32) - MUL_1892;
    let MUL_1909 = MUL_1778 * Simd::splat(0.17f32);
    let MUL_1912 = SUB_1786 * Simd::splat(0.17f32);
    let NEGATE_1913 = -MUL_1912;
    let MUL_1916 = MUL_1789 * Simd::splat(0.17f32);
    let SUB_1924 = Simd::splat(0.333f32) - MUL_1916;
    if
    /* panda_link2 */
    sphere_environment_in_collision(
        environment,
        ADD_1835,
        SUB_1836,
        ADD_1838,
        Simd::splat(0.145067f32),
    ) {
        if sphere_environment_in_collision(
            environment,
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
    } // (21, 99)
    let MUL_184 = ADD_130 * Simd::splat(0.7071068f32);
    let MUL_182 = SUB_149 * Simd::splat(0.7071068f32);
    let SUB_186 = MUL_182 - MUL_184;
    let ADD_215 = MUL_182 + MUL_184;
    let MUL_198 = ADD_144 * Simd::splat(0.7071068f32);
    let MUL_196 = SUB_138 * Simd::splat(0.7071068f32);
    let SUB_209 = MUL_198 - MUL_196;
    let ADD_199 = MUL_196 + MUL_198;
    let MUL_228 = ADD_130 * Simd::splat(0.316f32);
    let MUL_224 = ADD_144 * Simd::splat(0.316f32);
    let MUL_235 = SUB_149 * MUL_224;
    let MUL_236 = SUB_138 * MUL_228;
    let ADD_237 = MUL_235 + MUL_236;
    let MUL_240 = ADD_237 * Simd::splat(2.0f32);
    let INPUT_2 = q[2];
    let DIV_262 = INPUT_2 * Simd::splat(0.5f32);
    let SIN_263 = DIV_262.sin();
    let COS_269 = DIV_262.cos();
    let MUL_286 = ADD_215 * COS_269;
    let MUL_281 = ADD_215 * SIN_263;
    let MUL_284 = SUB_209 * COS_269;
    let ADD_285 = MUL_281 + MUL_284;
    let MUL_1936 = ADD_285 * ADD_285;
    let MUL_289 = SUB_209 * SIN_263;
    let SUB_290 = MUL_286 - MUL_289;
    let MUL_1937 = SUB_290 * ADD_285;
    let MUL_271 = SUB_186 * COS_269;
    let MUL_276 = SUB_186 * SIN_263;
    let MUL_278 = ADD_199 * COS_269;
    let SUB_279 = MUL_278 - MUL_276;
    let MUL_1938 = SUB_290 * SUB_279;
    let MUL_1935 = SUB_279 * SUB_279;
    let ADD_1944 = MUL_1935 + MUL_1936;
    let MUL_1947 = ADD_1944 * Simd::splat(2.0f32);
    let SUB_1950 = Simd::splat(1.0f32) - MUL_1947;
    let MUL_1984 = SUB_1950 * Simd::splat(0.0404726f32);
    let MUL_272 = ADD_199 * SIN_263;
    let ADD_273 = MUL_271 + MUL_272;
    let MUL_1942 = ADD_273 * ADD_285;
    let ADD_1970 = MUL_1942 + MUL_1938;
    let MUL_1972 = ADD_1970 * Simd::splat(2.0f32);
    let MUL_1997 = MUL_1972 * Simd::splat(0.0439448f32);
    let MUL_1941 = ADD_273 * SUB_279;
    let SUB_1957 = MUL_1941 - MUL_1937;
    let MUL_1959 = SUB_1957 * Simd::splat(2.0f32);
    let MUL_1990 = MUL_1959 * Simd::splat(0.0229569f32);
    let ADD_2007 = MUL_1984 + MUL_1990;
    let SUB_2010 = ADD_2007 - MUL_1997;
    let ADD_2013 = MUL_240 + SUB_2010;
    let ADD_1951 = MUL_1941 + MUL_1937;
    let MUL_244 = ADD_130 * MUL_228;
    let MUL_1953 = ADD_1951 * Simd::splat(2.0f32);
    let MUL_1986 = MUL_1953 * Simd::splat(0.0404726f32);
    let MUL_1940 = SUB_290 * ADD_273;
    let MUL_1943 = SUB_279 * ADD_285;
    let SUB_1973 = MUL_1943 - MUL_1940;
    let MUL_1975 = SUB_1973 * Simd::splat(2.0f32);
    let MUL_2001 = MUL_1975 * Simd::splat(0.0439448f32);
    let MUL_1939 = ADD_273 * ADD_273;
    let ADD_1960 = MUL_1936 + MUL_1939;
    let MUL_1963 = ADD_1960 * Simd::splat(2.0f32);
    let SUB_1966 = Simd::splat(1.0f32) - MUL_1963;
    let MUL_1992 = SUB_1966 * Simd::splat(0.0229569f32);
    let ADD_2008 = MUL_1986 + MUL_1992;
    let SUB_2011 = ADD_2008 - MUL_2001;
    let MUL_246 = ADD_144 * MUL_224;
    let ADD_247 = MUL_244 + MUL_246;
    let MUL_249 = ADD_247 * Simd::splat(2.0f32);
    let SUB_252 = MUL_249 - Simd::splat(0.316f32);
    let ADD_2014 = SUB_252 + SUB_2011;
    let SUB_1954 = MUL_1942 - MUL_1938;
    let ADD_1967 = MUL_1943 + MUL_1940;
    let ADD_1976 = MUL_1935 + MUL_1939;
    let MUL_1979 = ADD_1976 * Simd::splat(2.0f32);
    let SUB_1982 = Simd::splat(1.0f32) - MUL_1979;
    let MUL_2005 = SUB_1982 * Simd::splat(0.0439448f32);
    let MUL_1969 = ADD_1967 * Simd::splat(2.0f32);
    let MUL_1994 = MUL_1969 * Simd::splat(0.0229569f32);
    let MUL_1956 = SUB_1954 * Simd::splat(2.0f32);
    let MUL_1988 = MUL_1956 * Simd::splat(0.0404726f32);
    let ADD_2009 = MUL_1988 + MUL_1994;
    let SUB_2012 = ADD_2009 - MUL_2005;
    let MUL_253 = SUB_149 * MUL_228;
    let MUL_255 = SUB_138 * MUL_224;
    let SUB_256 = MUL_253 - MUL_255;
    let MUL_258 = SUB_256 * Simd::splat(2.0f32);
    let ADD_260 = Simd::splat(0.333f32) + MUL_258;
    let ADD_2015 = ADD_260 + SUB_2012;
    let MUL_2030 = MUL_1972 * Simd::splat(0.1f32);
    let SUB_2040 = MUL_240 - MUL_2030;
    let MUL_2034 = MUL_1975 * Simd::splat(0.1f32);
    let SUB_2041 = SUB_252 - MUL_2034;
    let MUL_2038 = SUB_1982 * Simd::splat(0.1f32);
    let SUB_2042 = ADD_260 - MUL_2038;
    let MUL_2057 = MUL_1972 * Simd::splat(0.06f32);
    let SUB_2067 = MUL_240 - MUL_2057;
    let MUL_2061 = MUL_1975 * Simd::splat(0.06f32);
    let SUB_2068 = SUB_252 - MUL_2061;
    let MUL_2065 = SUB_1982 * Simd::splat(0.06f32);
    let SUB_2069 = ADD_260 - MUL_2065;
    let MUL_2077 = MUL_1959 * Simd::splat(0.06f32);
    let MUL_2071 = SUB_1950 * Simd::splat(0.08f32);
    let ADD_2088 = MUL_2071 + MUL_2077;
    let ADD_2091 = MUL_240 + ADD_2088;
    let MUL_2079 = SUB_1966 * Simd::splat(0.06f32);
    let MUL_2073 = MUL_1953 * Simd::splat(0.08f32);
    let ADD_2089 = MUL_2073 + MUL_2079;
    let ADD_2092 = SUB_252 + ADD_2089;
    let MUL_2081 = MUL_1969 * Simd::splat(0.06f32);
    let MUL_2075 = MUL_1956 * Simd::splat(0.08f32);
    let ADD_2090 = MUL_2075 + MUL_2081;
    let ADD_2093 = ADD_260 + ADD_2090;
    let MUL_2101 = MUL_1959 * Simd::splat(0.02f32);
    let ADD_2112 = MUL_2071 + MUL_2101;
    let ADD_2115 = MUL_240 + ADD_2112;
    let MUL_2103 = SUB_1966 * Simd::splat(0.02f32);
    let ADD_2113 = MUL_2073 + MUL_2103;
    let ADD_2116 = SUB_252 + ADD_2113;
    let MUL_2105 = MUL_1969 * Simd::splat(0.02f32);
    let ADD_2114 = MUL_2075 + MUL_2105;
    let ADD_2117 = ADD_260 + ADD_2114;
    if
    /* panda_link3 */
    sphere_environment_in_collision(
        environment,
        ADD_2013,
        ADD_2014,
        ADD_2015,
        Simd::splat(0.127656f32),
    ) {
        if sphere_environment_in_collision(
            environment,
            SUB_2040,
            SUB_2041,
            SUB_2042,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            SUB_2067,
            SUB_2068,
            SUB_2069,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2091,
            ADD_2092,
            ADD_2093,
            Simd::splat(0.055f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2115,
            ADD_2116,
            ADD_2117,
            Simd::splat(0.055f32),
        ) {
            return false;
        }
    } // (99, 220)
    let MUL_323 = SUB_290 * Simd::splat(0.7071068f32);
    let MUL_338 = ADD_285 * Simd::splat(0.7071068f32);
    let MUL_336 = SUB_279 * Simd::splat(0.7071068f32);
    let SUB_349 = MUL_338 - MUL_336;
    let ADD_339 = MUL_336 + MUL_338;
    let MUL_325 = ADD_273 * Simd::splat(0.7071068f32);
    let SUB_354 = MUL_323 - MUL_325;
    let ADD_326 = MUL_323 + MUL_325;
    let MUL_371 = ADD_285 * Simd::splat(0.0825f32);
    let MUL_376 = ADD_285 * MUL_371;
    let MUL_366 = SUB_279 * Simd::splat(0.0825f32);
    let MUL_374 = SUB_279 * MUL_366;
    let ADD_378 = MUL_374 + MUL_376;
    let MUL_381 = ADD_378 * Simd::splat(2.0f32);
    let SUB_384 = Simd::splat(0.0825f32) - MUL_381;
    let ADD_403 = MUL_240 + SUB_384;
    let INPUT_3 = q[3];
    let DIV_407 = INPUT_3 * Simd::splat(0.5f32);
    let SIN_408 = DIV_407.sin();
    let COS_414 = DIV_407.cos();
    let MUL_431 = SUB_354 * COS_414;
    let MUL_426 = SUB_354 * SIN_408;
    let MUL_429 = SUB_349 * COS_414;
    let ADD_430 = MUL_426 + MUL_429;
    let MUL_2129 = ADD_430 * ADD_430;
    let MUL_434 = SUB_349 * SIN_408;
    let SUB_435 = MUL_431 - MUL_434;
    let MUL_2130 = SUB_435 * ADD_430;
    let MUL_416 = ADD_326 * COS_414;
    let MUL_421 = ADD_326 * SIN_408;
    let MUL_423 = ADD_339 * COS_414;
    let SUB_424 = MUL_423 - MUL_421;
    let MUL_2131 = SUB_435 * SUB_424;
    let MUL_2128 = SUB_424 * SUB_424;
    let ADD_2137 = MUL_2128 + MUL_2129;
    let MUL_2140 = ADD_2137 * Simd::splat(2.0f32);
    let SUB_2143 = Simd::splat(1.0f32) - MUL_2140;
    let MUL_2178 = SUB_2143 * Simd::splat(0.0422068f32);
    let MUL_417 = ADD_339 * SIN_408;
    let ADD_418 = MUL_416 + MUL_417;
    let MUL_2135 = ADD_418 * ADD_430;
    let ADD_2163 = MUL_2135 + MUL_2131;
    let MUL_2165 = ADD_2163 * Simd::splat(2.0f32);
    let MUL_2195 = MUL_2165 * Simd::splat(0.0226917f32);
    let MUL_2134 = ADD_418 * SUB_424;
    let SUB_2150 = MUL_2134 - MUL_2130;
    let MUL_2152 = SUB_2150 * Simd::splat(2.0f32);
    let MUL_2189 = MUL_2152 * Simd::splat(0.0449876f32);
    let SUB_2200 = MUL_2189 - MUL_2178;
    let ADD_2203 = SUB_2200 + MUL_2195;
    let ADD_2206 = ADD_403 + ADD_2203;
    let ADD_2144 = MUL_2134 + MUL_2130;
    let MUL_2146 = ADD_2144 * Simd::splat(2.0f32);
    let MUL_2182 = MUL_2146 * Simd::splat(0.0422068f32);
    let MUL_2133 = SUB_435 * ADD_418;
    let MUL_2136 = SUB_424 * ADD_430;
    let SUB_2166 = MUL_2136 - MUL_2133;
    let MUL_2168 = SUB_2166 * Simd::splat(2.0f32);
    let MUL_2197 = MUL_2168 * Simd::splat(0.0226917f32);
    let MUL_2132 = ADD_418 * ADD_418;
    let ADD_2153 = MUL_2129 + MUL_2132;
    let MUL_2156 = ADD_2153 * Simd::splat(2.0f32);
    let SUB_2159 = Simd::splat(1.0f32) - MUL_2156;
    let MUL_2191 = SUB_2159 * Simd::splat(0.0449876f32);
    let SUB_2201 = MUL_2191 - MUL_2182;
    let ADD_2204 = SUB_2201 + MUL_2197;
    let MUL_386 = SUB_290 * MUL_371;
    let MUL_387 = ADD_273 * MUL_366;
    let ADD_389 = MUL_386 + MUL_387;
    let MUL_392 = ADD_389 * Simd::splat(2.0f32);
    let ADD_404 = SUB_252 + MUL_392;
    let ADD_2207 = ADD_404 + ADD_2204;
    let SUB_2147 = MUL_2135 - MUL_2131;
    let ADD_2160 = MUL_2136 + MUL_2133;
    let ADD_2169 = MUL_2128 + MUL_2132;
    let MUL_2172 = ADD_2169 * Simd::splat(2.0f32);
    let SUB_2175 = Simd::splat(1.0f32) - MUL_2172;
    let MUL_2199 = SUB_2175 * Simd::splat(0.0226917f32);
    let MUL_2162 = ADD_2160 * Simd::splat(2.0f32);
    let MUL_2193 = MUL_2162 * Simd::splat(0.0449876f32);
    let MUL_2149 = SUB_2147 * Simd::splat(2.0f32);
    let MUL_2186 = MUL_2149 * Simd::splat(0.0422068f32);
    let SUB_2202 = MUL_2193 - MUL_2186;
    let ADD_2205 = SUB_2202 + MUL_2199;
    let MUL_394 = SUB_290 * MUL_366;
    let MUL_396 = ADD_273 * MUL_371;
    let SUB_398 = MUL_396 - MUL_394;
    let MUL_401 = SUB_398 * Simd::splat(2.0f32);
    let ADD_405 = ADD_260 + MUL_401;
    let ADD_2208 = ADD_405 + ADD_2205;
    let MUL_2211 = SUB_2143 * Simd::splat(0.08f32);
    let MUL_2222 = MUL_2152 * Simd::splat(0.095f32);
    let SUB_2233 = MUL_2222 - MUL_2211;
    let ADD_2236 = ADD_403 + SUB_2233;
    let MUL_2224 = SUB_2159 * Simd::splat(0.095f32);
    let MUL_2215 = MUL_2146 * Simd::splat(0.08f32);
    let SUB_2234 = MUL_2224 - MUL_2215;
    let ADD_2237 = ADD_404 + SUB_2234;
    let MUL_2226 = MUL_2162 * Simd::splat(0.095f32);
    let MUL_2219 = MUL_2149 * Simd::splat(0.08f32);
    let SUB_2235 = MUL_2226 - MUL_2219;
    let ADD_2238 = ADD_405 + SUB_2235;
    let MUL_2252 = MUL_2165 * Simd::splat(0.02f32);
    let ADD_2257 = ADD_403 + MUL_2252;
    let MUL_2254 = MUL_2168 * Simd::splat(0.02f32);
    let ADD_2258 = ADD_404 + MUL_2254;
    let MUL_2256 = SUB_2175 * Simd::splat(0.02f32);
    let ADD_2259 = ADD_405 + MUL_2256;
    let MUL_2273 = MUL_2165 * Simd::splat(0.06f32);
    let ADD_2278 = ADD_403 + MUL_2273;
    let MUL_2275 = MUL_2168 * Simd::splat(0.06f32);
    let ADD_2279 = ADD_404 + MUL_2275;
    let MUL_2277 = SUB_2175 * Simd::splat(0.06f32);
    let ADD_2280 = ADD_405 + MUL_2277;
    let MUL_2294 = MUL_2152 * Simd::splat(0.06f32);
    let SUB_2305 = MUL_2294 - MUL_2211;
    let ADD_2308 = ADD_403 + SUB_2305;
    let MUL_2296 = SUB_2159 * Simd::splat(0.06f32);
    let SUB_2306 = MUL_2296 - MUL_2215;
    let ADD_2309 = ADD_404 + SUB_2306;
    let MUL_2298 = MUL_2162 * Simd::splat(0.06f32);
    let SUB_2307 = MUL_2298 - MUL_2219;
    let ADD_2310 = ADD_405 + SUB_2307;
    if
    /* panda_link4 */
    sphere_environment_in_collision(
        environment,
        ADD_2206,
        ADD_2207,
        ADD_2208,
        Simd::splat(0.12849f32),
    ) {
        if sphere_environment_in_collision(
            environment,
            ADD_2236,
            ADD_2237,
            ADD_2238,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2257,
            ADD_2258,
            ADD_2259,
            Simd::splat(0.055f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2278,
            ADD_2279,
            ADD_2280,
            Simd::splat(0.055f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2308,
            ADD_2309,
            ADD_2310,
            Simd::splat(0.055f32),
        ) {
            return false;
        }
    } // (220, 343)
    let MUL_469 = SUB_435 * Simd::splat(0.7071068f32);
    let MUL_486 = ADD_430 * Simd::splat(0.7071068f32);
    let MUL_527 = ADD_430 * Simd::splat(0.0825f32);
    let MUL_533 = ADD_430 * MUL_527;
    let MUL_483 = SUB_424 * Simd::splat(0.7071068f32);
    let SUB_488 = MUL_483 - MUL_486;
    let ADD_499 = MUL_483 + MUL_486;
    let MUL_520 = SUB_424 * Simd::splat(0.0825f32);
    let MUL_472 = ADD_418 * Simd::splat(0.7071068f32);
    let SUB_473 = MUL_472 - MUL_469;
    let ADD_506 = MUL_469 + MUL_472;
    let MUL_514 = ADD_430 * Simd::splat(0.384f32);
    let MUL_529 = SUB_435 * MUL_514;
    let MUL_517 = ADD_418 * Simd::splat(0.384f32);
    let ADD_522 = MUL_517 + MUL_520;
    let MUL_531 = SUB_424 * ADD_522;
    let SUB_532 = MUL_531 - MUL_529;
    let ADD_534 = SUB_532 + MUL_533;
    let MUL_536 = ADD_534 * Simd::splat(2.0f32);
    let SUB_539 = MUL_536 - Simd::splat(0.0825f32);
    let ADD_564 = ADD_403 + SUB_539;
    let INPUT_4 = q[4];
    let DIV_568 = INPUT_4 * Simd::splat(0.5f32);
    let SIN_569 = DIV_568.sin();
    let COS_575 = DIV_568.cos();
    let MUL_592 = ADD_506 * COS_575;
    let MUL_587 = ADD_506 * SIN_569;
    let MUL_590 = ADD_499 * COS_575;
    let ADD_591 = MUL_587 + MUL_590;
    let MUL_2322 = ADD_591 * ADD_591;
    let MUL_595 = ADD_499 * SIN_569;
    let SUB_596 = MUL_592 - MUL_595;
    let MUL_2323 = SUB_596 * ADD_591;
    let MUL_577 = SUB_473 * COS_575;
    let MUL_582 = SUB_473 * SIN_569;
    let MUL_584 = SUB_488 * COS_575;
    let SUB_585 = MUL_584 - MUL_582;
    let MUL_2324 = SUB_596 * SUB_585;
    let MUL_2321 = SUB_585 * SUB_585;
    let ADD_2330 = MUL_2321 + MUL_2322;
    let MUL_2333 = ADD_2330 * Simd::splat(2.0f32);
    let SUB_2336 = Simd::splat(1.0f32) - MUL_2333;
    let MUL_2370 = SUB_2336 * Simd::splat(3.14e-05f32);
    let MUL_578 = SUB_488 * SIN_569;
    let ADD_579 = MUL_577 + MUL_578;
    let MUL_2328 = ADD_579 * ADD_591;
    let ADD_2356 = MUL_2328 + MUL_2324;
    let MUL_2358 = ADD_2356 * Simd::splat(2.0f32);
    let MUL_2383 = MUL_2358 * Simd::splat(0.1065559f32);
    let MUL_2327 = ADD_579 * SUB_585;
    let SUB_2343 = MUL_2327 - MUL_2323;
    let MUL_2345 = SUB_2343 * Simd::splat(2.0f32);
    let MUL_2376 = MUL_2345 * Simd::splat(0.027819f32);
    let ADD_2393 = MUL_2370 + MUL_2376;
    let SUB_2396 = ADD_2393 - MUL_2383;
    let ADD_2399 = ADD_564 + SUB_2396;
    let ADD_2337 = MUL_2327 + MUL_2323;
    let MUL_2339 = ADD_2337 * Simd::splat(2.0f32);
    let MUL_2372 = MUL_2339 * Simd::splat(3.14e-05f32);
    let MUL_2326 = SUB_596 * ADD_579;
    let MUL_2329 = SUB_585 * ADD_591;
    let SUB_2359 = MUL_2329 - MUL_2326;
    let MUL_2361 = SUB_2359 * Simd::splat(2.0f32);
    let MUL_2387 = MUL_2361 * Simd::splat(0.1065559f32);
    let MUL_2325 = ADD_579 * ADD_579;
    let ADD_2346 = MUL_2322 + MUL_2325;
    let MUL_2349 = ADD_2346 * Simd::splat(2.0f32);
    let SUB_2352 = Simd::splat(1.0f32) - MUL_2349;
    let MUL_2378 = SUB_2352 * Simd::splat(0.027819f32);
    let ADD_2394 = MUL_2372 + MUL_2378;
    let SUB_2397 = ADD_2394 - MUL_2387;
    let MUL_541 = SUB_435 * MUL_527;
    let MUL_546 = ADD_430 * MUL_514;
    let MUL_543 = ADD_418 * ADD_522;
    let ADD_544 = MUL_541 + MUL_543;
    let ADD_548 = ADD_544 + MUL_546;
    let MUL_551 = ADD_548 * Simd::splat(2.0f32);
    let SUB_554 = Simd::splat(0.384f32) - MUL_551;
    let ADD_565 = ADD_404 + SUB_554;
    let ADD_2400 = ADD_565 + SUB_2397;
    let SUB_2340 = MUL_2328 - MUL_2324;
    let ADD_2353 = MUL_2329 + MUL_2326;
    let ADD_2362 = MUL_2321 + MUL_2325;
    let MUL_2365 = ADD_2362 * Simd::splat(2.0f32);
    let SUB_2368 = Simd::splat(1.0f32) - MUL_2365;
    let MUL_2391 = SUB_2368 * Simd::splat(0.1065559f32);
    let MUL_2355 = ADD_2353 * Simd::splat(2.0f32);
    let MUL_2380 = MUL_2355 * Simd::splat(0.027819f32);
    let MUL_2342 = SUB_2340 * Simd::splat(2.0f32);
    let MUL_2374 = MUL_2342 * Simd::splat(3.14e-05f32);
    let ADD_2395 = MUL_2374 + MUL_2380;
    let SUB_2398 = ADD_2395 - MUL_2391;
    let MUL_555 = SUB_435 * ADD_522;
    let MUL_558 = SUB_424 * MUL_514;
    let MUL_556 = ADD_418 * MUL_527;
    let SUB_557 = MUL_555 - MUL_556;
    let ADD_560 = SUB_557 + MUL_558;
    let MUL_562 = ADD_560 * Simd::splat(2.0f32);
    let ADD_566 = ADD_405 + MUL_562;
    let ADD_2401 = ADD_566 + SUB_2398;
    let MUL_2409 = MUL_2345 * Simd::splat(0.055f32);
    let ADD_2420 = ADD_564 + MUL_2409;
    let MUL_2411 = SUB_2352 * Simd::splat(0.055f32);
    let ADD_2421 = ADD_565 + MUL_2411;
    let MUL_2413 = MUL_2355 * Simd::splat(0.055f32);
    let ADD_2422 = ADD_566 + MUL_2413;
    let MUL_2430 = MUL_2345 * Simd::splat(0.075f32);
    let ADD_2441 = ADD_564 + MUL_2430;
    let MUL_2432 = SUB_2352 * Simd::splat(0.075f32);
    let ADD_2442 = ADD_565 + MUL_2432;
    let MUL_2434 = MUL_2355 * Simd::splat(0.075f32);
    let ADD_2443 = ADD_566 + MUL_2434;
    let MUL_2458 = MUL_2358 * Simd::splat(0.22f32);
    let SUB_2468 = ADD_564 - MUL_2458;
    let MUL_2462 = MUL_2361 * Simd::splat(0.22f32);
    let SUB_2469 = ADD_565 - MUL_2462;
    let MUL_2466 = SUB_2368 * Simd::splat(0.22f32);
    let SUB_2470 = ADD_566 - MUL_2466;
    let MUL_2485 = MUL_2358 * Simd::splat(0.18f32);
    let MUL_2478 = MUL_2345 * Simd::splat(0.05f32);
    let SUB_2495 = MUL_2478 - MUL_2485;
    let ADD_2498 = ADD_564 + SUB_2495;
    let MUL_2489 = MUL_2361 * Simd::splat(0.18f32);
    let MUL_2480 = SUB_2352 * Simd::splat(0.05f32);
    let SUB_2496 = MUL_2480 - MUL_2489;
    let ADD_2499 = ADD_565 + SUB_2496;
    let MUL_2493 = SUB_2368 * Simd::splat(0.18f32);
    let MUL_2482 = MUL_2355 * Simd::splat(0.05f32);
    let SUB_2497 = MUL_2482 - MUL_2493;
    let ADD_2500 = ADD_566 + SUB_2497;
    let MUL_2508 = MUL_2345 * Simd::splat(0.08f32);
    let MUL_2515 = MUL_2358 * Simd::splat(0.14f32);
    let MUL_2502 = SUB_2336 * Simd::splat(0.01f32);
    let ADD_2525 = MUL_2502 + MUL_2508;
    let SUB_2528 = ADD_2525 - MUL_2515;
    let ADD_2531 = ADD_564 + SUB_2528;
    let MUL_2519 = MUL_2361 * Simd::splat(0.14f32);
    let MUL_2510 = SUB_2352 * Simd::splat(0.08f32);
    let MUL_2504 = MUL_2339 * Simd::splat(0.01f32);
    let ADD_2526 = MUL_2504 + MUL_2510;
    let SUB_2529 = ADD_2526 - MUL_2519;
    let ADD_2532 = ADD_565 + SUB_2529;
    let MUL_2523 = SUB_2368 * Simd::splat(0.14f32);
    let MUL_2512 = MUL_2355 * Simd::splat(0.08f32);
    let MUL_2506 = MUL_2342 * Simd::splat(0.01f32);
    let ADD_2527 = MUL_2506 + MUL_2512;
    let SUB_2530 = ADD_2527 - MUL_2523;
    let ADD_2533 = ADD_566 + SUB_2530;
    let MUL_2548 = MUL_2358 * Simd::splat(0.11f32);
    let MUL_2541 = MUL_2345 * Simd::splat(0.085f32);
    let ADD_2558 = MUL_2502 + MUL_2541;
    let SUB_2561 = ADD_2558 - MUL_2548;
    let ADD_2564 = ADD_564 + SUB_2561;
    let MUL_2552 = MUL_2361 * Simd::splat(0.11f32);
    let MUL_2543 = SUB_2352 * Simd::splat(0.085f32);
    let ADD_2559 = MUL_2504 + MUL_2543;
    let SUB_2562 = ADD_2559 - MUL_2552;
    let ADD_2565 = ADD_565 + SUB_2562;
    let MUL_2556 = SUB_2368 * Simd::splat(0.11f32);
    let MUL_2545 = MUL_2355 * Simd::splat(0.085f32);
    let ADD_2560 = MUL_2506 + MUL_2545;
    let SUB_2563 = ADD_2560 - MUL_2556;
    let ADD_2566 = ADD_566 + SUB_2563;
    let MUL_2581 = MUL_2358 * Simd::splat(0.08f32);
    let MUL_2574 = MUL_2345 * Simd::splat(0.09f32);
    let ADD_2591 = MUL_2502 + MUL_2574;
    let SUB_2594 = ADD_2591 - MUL_2581;
    let ADD_2597 = ADD_564 + SUB_2594;
    let MUL_2585 = MUL_2361 * Simd::splat(0.08f32);
    let MUL_2576 = SUB_2352 * Simd::splat(0.09f32);
    let ADD_2592 = MUL_2504 + MUL_2576;
    let SUB_2595 = ADD_2592 - MUL_2585;
    let ADD_2598 = ADD_565 + SUB_2595;
    let MUL_2589 = SUB_2368 * Simd::splat(0.08f32);
    let MUL_2578 = MUL_2355 * Simd::splat(0.09f32);
    let ADD_2593 = MUL_2506 + MUL_2578;
    let SUB_2596 = ADD_2593 - MUL_2589;
    let ADD_2599 = ADD_566 + SUB_2596;
    let MUL_2614 = MUL_2358 * Simd::splat(0.05f32);
    let MUL_2607 = MUL_2345 * Simd::splat(0.095f32);
    let ADD_2624 = MUL_2502 + MUL_2607;
    let SUB_2627 = ADD_2624 - MUL_2614;
    let ADD_2630 = ADD_564 + SUB_2627;
    let MUL_2618 = MUL_2361 * Simd::splat(0.05f32);
    let MUL_2609 = SUB_2352 * Simd::splat(0.095f32);
    let ADD_2625 = MUL_2504 + MUL_2609;
    let SUB_2628 = ADD_2625 - MUL_2618;
    let ADD_2631 = ADD_565 + SUB_2628;
    let MUL_2622 = SUB_2368 * Simd::splat(0.05f32);
    let MUL_2611 = MUL_2355 * Simd::splat(0.095f32);
    let ADD_2626 = MUL_2506 + MUL_2611;
    let SUB_2629 = ADD_2626 - MUL_2622;
    let ADD_2632 = ADD_566 + SUB_2629;
    let SUB_2663 = MUL_2508 - MUL_2502;
    let SUB_2666 = SUB_2663 - MUL_2515;
    let ADD_2669 = ADD_564 + SUB_2666;
    let SUB_2664 = MUL_2510 - MUL_2504;
    let SUB_2667 = SUB_2664 - MUL_2519;
    let ADD_2670 = ADD_565 + SUB_2667;
    let SUB_2665 = MUL_2512 - MUL_2506;
    let SUB_2668 = SUB_2665 - MUL_2523;
    let ADD_2671 = ADD_566 + SUB_2668;
    let SUB_2702 = MUL_2541 - MUL_2502;
    let SUB_2705 = SUB_2702 - MUL_2548;
    let ADD_2708 = ADD_564 + SUB_2705;
    let SUB_2703 = MUL_2543 - MUL_2504;
    let SUB_2706 = SUB_2703 - MUL_2552;
    let ADD_2709 = ADD_565 + SUB_2706;
    let SUB_2704 = MUL_2545 - MUL_2506;
    let SUB_2707 = SUB_2704 - MUL_2556;
    let ADD_2710 = ADD_566 + SUB_2707;
    let SUB_2741 = MUL_2574 - MUL_2502;
    let SUB_2744 = SUB_2741 - MUL_2581;
    let ADD_2747 = ADD_564 + SUB_2744;
    let SUB_2742 = MUL_2576 - MUL_2504;
    let SUB_2745 = SUB_2742 - MUL_2585;
    let ADD_2748 = ADD_565 + SUB_2745;
    let SUB_2743 = MUL_2578 - MUL_2506;
    let SUB_2746 = SUB_2743 - MUL_2589;
    let ADD_2749 = ADD_566 + SUB_2746;
    let SUB_2780 = MUL_2607 - MUL_2502;
    let SUB_2783 = SUB_2780 - MUL_2614;
    let ADD_2786 = ADD_564 + SUB_2783;
    let SUB_2781 = MUL_2609 - MUL_2504;
    let SUB_2784 = SUB_2781 - MUL_2618;
    let ADD_2787 = ADD_565 + SUB_2784;
    let SUB_2782 = MUL_2611 - MUL_2506;
    let SUB_2785 = SUB_2782 - MUL_2622;
    let ADD_2788 = ADD_566 + SUB_2785;
    if
    /* panda_link0 vs. panda_link5 */
    sphere_sphere_self_collision(
        Simd::splat(-0.043343f32),
        Simd::splat(1.4e-06f32),
        Simd::splat(0.0629063f32),
        Simd::splat(0.130366f32),
        ADD_2399,
        ADD_2400,
        ADD_2401,
        Simd::splat(0.173531f32),
    ) {
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
    } // (343, 572)
    if
    /* panda_link1 vs. panda_link5 */
    sphere_sphere_self_collision(
        ADD_1636,
        SUB_1637,
        Simd::splat(0.2598976f32),
        Simd::splat(0.144259f32),
        ADD_2399,
        ADD_2400,
        ADD_2401,
        Simd::splat(0.173531f32),
    ) {
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
    } // (572, 572)
    if
    /* panda_link2 vs. panda_link5 */
    sphere_sphere_self_collision(
        ADD_1835,
        SUB_1836,
        ADD_1838,
        Simd::splat(0.145067f32),
        ADD_2399,
        ADD_2400,
        ADD_2401,
        Simd::splat(0.173531f32),
    ) {
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
    } // (572, 572)
    if
    /* panda_link5 */
    sphere_environment_in_collision(
        environment,
        ADD_2399,
        ADD_2400,
        ADD_2401,
        Simd::splat(0.173531f32),
    ) {
        if sphere_environment_in_collision(
            environment,
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
    } // (572, 572)
    let MUL_629 = SUB_596 * Simd::splat(0.7071068f32);
    let MUL_644 = ADD_591 * Simd::splat(0.7071068f32);
    let MUL_642 = SUB_585 * Simd::splat(0.7071068f32);
    let SUB_655 = MUL_644 - MUL_642;
    let ADD_645 = MUL_642 + MUL_644;
    let MUL_631 = ADD_579 * Simd::splat(0.7071068f32);
    let SUB_660 = MUL_629 - MUL_631;
    let ADD_632 = MUL_629 + MUL_631;
    let INPUT_5 = q[5];
    let DIV_697 = INPUT_5 * Simd::splat(0.5f32);
    let SIN_698 = DIV_697.sin();
    let COS_704 = DIV_697.cos();
    let MUL_716 = SUB_660 * SIN_698;
    let MUL_721 = SUB_660 * COS_704;
    let MUL_724 = SUB_655 * SIN_698;
    let SUB_725 = MUL_721 - MUL_724;
    let MUL_719 = SUB_655 * COS_704;
    let ADD_720 = MUL_716 + MUL_719;
    let MUL_2817 = SUB_725 * ADD_720;
    let MUL_2816 = ADD_720 * ADD_720;
    let MUL_711 = ADD_632 * SIN_698;
    let MUL_706 = ADD_632 * COS_704;
    let MUL_707 = ADD_645 * SIN_698;
    let ADD_708 = MUL_706 + MUL_707;
    let MUL_2822 = ADD_708 * ADD_720;
    let MUL_713 = ADD_645 * COS_704;
    let SUB_714 = MUL_713 - MUL_711;
    let MUL_2818 = SUB_725 * SUB_714;
    let ADD_2850 = MUL_2822 + MUL_2818;
    let MUL_2852 = ADD_2850 * Simd::splat(2.0f32);
    let MUL_2876 = MUL_2852 * Simd::splat(0.0108239f32);
    let MUL_2815 = SUB_714 * SUB_714;
    let ADD_2824 = MUL_2815 + MUL_2816;
    let MUL_2827 = ADD_2824 * Simd::splat(2.0f32);
    let SUB_2830 = Simd::splat(1.0f32) - MUL_2827;
    let MUL_2864 = SUB_2830 * Simd::splat(0.0485274f32);
    let MUL_2821 = ADD_708 * SUB_714;
    let SUB_2837 = MUL_2821 - MUL_2817;
    let MUL_2839 = SUB_2837 * Simd::splat(2.0f32);
    let MUL_2870 = MUL_2839 * Simd::splat(0.0033602f32);
    let ADD_2881 = MUL_2864 + MUL_2870;
    let ADD_2884 = ADD_2881 + MUL_2876;
    let ADD_2887 = ADD_564 + ADD_2884;
    let ADD_2831 = MUL_2821 + MUL_2817;
    let MUL_2833 = ADD_2831 * Simd::splat(2.0f32);
    let MUL_2866 = MUL_2833 * Simd::splat(0.0485274f32);
    let MUL_2820 = SUB_725 * ADD_708;
    let MUL_2823 = SUB_714 * ADD_720;
    let SUB_2853 = MUL_2823 - MUL_2820;
    let MUL_2855 = SUB_2853 * Simd::splat(2.0f32);
    let MUL_2878 = MUL_2855 * Simd::splat(0.0108239f32);
    let MUL_2819 = ADD_708 * ADD_708;
    let ADD_2840 = MUL_2816 + MUL_2819;
    let MUL_2843 = ADD_2840 * Simd::splat(2.0f32);
    let SUB_2846 = Simd::splat(1.0f32) - MUL_2843;
    let MUL_2872 = SUB_2846 * Simd::splat(0.0033602f32);
    let ADD_2882 = MUL_2866 + MUL_2872;
    let ADD_2885 = ADD_2882 + MUL_2878;
    let ADD_2888 = ADD_565 + ADD_2885;
    let SUB_2834 = MUL_2822 - MUL_2818;
    let ADD_2847 = MUL_2823 + MUL_2820;
    let ADD_2856 = MUL_2815 + MUL_2819;
    let MUL_2859 = ADD_2856 * Simd::splat(2.0f32);
    let SUB_2862 = Simd::splat(1.0f32) - MUL_2859;
    let MUL_2880 = SUB_2862 * Simd::splat(0.0108239f32);
    let MUL_2849 = ADD_2847 * Simd::splat(2.0f32);
    let MUL_2874 = MUL_2849 * Simd::splat(0.0033602f32);
    let MUL_2836 = SUB_2834 * Simd::splat(2.0f32);
    let MUL_2868 = MUL_2836 * Simd::splat(0.0485274f32);
    let ADD_2883 = MUL_2868 + MUL_2874;
    let ADD_2886 = ADD_2883 + MUL_2880;
    let ADD_2889 = ADD_566 + ADD_2886;
    let MUL_2916 = MUL_2839 * Simd::splat(0.01f32);
    let MUL_2909 = SUB_2830 * Simd::splat(0.08f32);
    let SUB_2932 = MUL_2909 - MUL_2916;
    let ADD_2935 = ADD_564 + SUB_2932;
    let MUL_2920 = SUB_2846 * Simd::splat(0.01f32);
    let MUL_2911 = MUL_2833 * Simd::splat(0.08f32);
    let SUB_2933 = MUL_2911 - MUL_2920;
    let ADD_2936 = ADD_565 + SUB_2933;
    let MUL_2924 = MUL_2849 * Simd::splat(0.01f32);
    let MUL_2913 = MUL_2836 * Simd::splat(0.08f32);
    let SUB_2934 = MUL_2913 - MUL_2924;
    let ADD_2937 = ADD_566 + SUB_2934;
    let MUL_2945 = MUL_2839 * Simd::splat(0.035f32);
    let ADD_2956 = MUL_2909 + MUL_2945;
    let ADD_2959 = ADD_564 + ADD_2956;
    let MUL_2947 = SUB_2846 * Simd::splat(0.035f32);
    let ADD_2957 = MUL_2911 + MUL_2947;
    let ADD_2960 = ADD_565 + ADD_2957;
    let MUL_2949 = MUL_2849 * Simd::splat(0.035f32);
    let ADD_2958 = MUL_2913 + MUL_2949;
    let ADD_2961 = ADD_566 + ADD_2958;
    if
    /* panda_link0 vs. panda_link6 */
    sphere_sphere_self_collision(
        Simd::splat(-0.043343f32),
        Simd::splat(1.4e-06f32),
        Simd::splat(0.0629063f32),
        Simd::splat(0.130366f32),
        ADD_2887,
        ADD_2888,
        ADD_2889,
        Simd::splat(0.104795f32),
    ) {
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_564,
            ADD_565,
            ADD_566,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_2935,
            ADD_2936,
            ADD_2937,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_2959,
            ADD_2960,
            ADD_2961,
            Simd::splat(0.052f32),
        ) {
            return false;
        }
    } // (572, 665)
    if
    /* panda_link1 vs. panda_link6 */
    sphere_sphere_self_collision(
        ADD_1636,
        SUB_1637,
        Simd::splat(0.2598976f32),
        Simd::splat(0.144259f32),
        ADD_2887,
        ADD_2888,
        ADD_2889,
        Simd::splat(0.104795f32),
    ) {
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_564,
            ADD_565,
            ADD_566,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2935,
            ADD_2936,
            ADD_2937,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2959,
            ADD_2960,
            ADD_2961,
            Simd::splat(0.052f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_564,
            ADD_565,
            ADD_566,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2935,
            ADD_2936,
            ADD_2937,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_2959,
            ADD_2960,
            ADD_2961,
            Simd::splat(0.052f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_564,
            ADD_565,
            ADD_566,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_2935,
            ADD_2936,
            ADD_2937,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_2959,
            ADD_2960,
            ADD_2961,
            Simd::splat(0.052f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_564,
            ADD_565,
            ADD_566,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_2935,
            ADD_2936,
            ADD_2937,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_2959,
            ADD_2960,
            ADD_2961,
            Simd::splat(0.052f32),
        ) {
            return false;
        }
    } // (665, 665)
    if
    /* panda_link6 */
    sphere_environment_in_collision(
        environment,
        ADD_2887,
        ADD_2888,
        ADD_2889,
        Simd::splat(0.104795f32),
    ) {
        if sphere_environment_in_collision(
            environment,
            ADD_564,
            ADD_565,
            ADD_566,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2935,
            ADD_2936,
            ADD_2937,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_2959,
            ADD_2960,
            ADD_2961,
            Simd::splat(0.052f32),
        ) {
            return false;
        }
    } // (665, 665)
    let MUL_758 = SUB_725 * Simd::splat(0.7071068f32);
    let MUL_773 = ADD_720 * Simd::splat(0.7071068f32);
    let MUL_771 = SUB_714 * Simd::splat(0.7071068f32);
    let SUB_784 = MUL_773 - MUL_771;
    let ADD_774 = MUL_771 + MUL_773;
    let MUL_760 = ADD_708 * Simd::splat(0.7071068f32);
    let SUB_789 = MUL_758 - MUL_760;
    let ADD_761 = MUL_758 + MUL_760;
    let MUL_806 = ADD_720 * Simd::splat(0.088f32);
    let MUL_811 = ADD_720 * MUL_806;
    let MUL_801 = SUB_714 * Simd::splat(0.088f32);
    let MUL_809 = SUB_714 * MUL_801;
    let ADD_813 = MUL_809 + MUL_811;
    let MUL_816 = ADD_813 * Simd::splat(2.0f32);
    let SUB_819 = Simd::splat(0.088f32) - MUL_816;
    let ADD_838 = ADD_564 + SUB_819;
    let INPUT_6 = q[6];
    let DIV_842 = INPUT_6 * Simd::splat(0.5f32);
    let SIN_843 = DIV_842.sin();
    let COS_849 = DIV_842.cos();
    let MUL_866 = SUB_789 * COS_849;
    let MUL_861 = SUB_789 * SIN_843;
    let MUL_864 = SUB_784 * COS_849;
    let ADD_865 = MUL_861 + MUL_864;
    let MUL_2971 = ADD_865 * ADD_865;
    let MUL_869 = SUB_784 * SIN_843;
    let SUB_870 = MUL_866 - MUL_869;
    let MUL_2972 = SUB_870 * ADD_865;
    let MUL_851 = ADD_761 * COS_849;
    let MUL_856 = ADD_761 * SIN_843;
    let MUL_858 = ADD_774 * COS_849;
    let SUB_859 = MUL_858 - MUL_856;
    let MUL_2973 = SUB_870 * SUB_859;
    let MUL_2970 = SUB_859 * SUB_859;
    let ADD_2979 = MUL_2970 + MUL_2971;
    let MUL_2982 = ADD_2979 * Simd::splat(2.0f32);
    let SUB_2985 = Simd::splat(1.0f32) - MUL_2982;
    let MUL_3019 = SUB_2985 * Simd::splat(0.0132698f32);
    let MUL_852 = ADD_774 * SIN_843;
    let ADD_853 = MUL_851 + MUL_852;
    let MUL_2977 = ADD_853 * ADD_865;
    let ADD_3005 = MUL_2977 + MUL_2973;
    let MUL_3007 = ADD_3005 * Simd::splat(2.0f32);
    let MUL_3031 = MUL_3007 * Simd::splat(0.0793978f32);
    let MUL_2976 = ADD_853 * SUB_859;
    let SUB_2992 = MUL_2976 - MUL_2972;
    let MUL_2994 = SUB_2992 * Simd::splat(2.0f32);
    let MUL_3025 = MUL_2994 * Simd::splat(0.0133909f32);
    let ADD_3036 = MUL_3019 + MUL_3025;
    let ADD_3039 = ADD_3036 + MUL_3031;
    let ADD_3042 = ADD_838 + ADD_3039;
    let ADD_2986 = MUL_2976 + MUL_2972;
    let MUL_2988 = ADD_2986 * Simd::splat(2.0f32);
    let MUL_3021 = MUL_2988 * Simd::splat(0.0132698f32);
    let MUL_2975 = SUB_870 * ADD_853;
    let MUL_2978 = SUB_859 * ADD_865;
    let SUB_3008 = MUL_2978 - MUL_2975;
    let MUL_3010 = SUB_3008 * Simd::splat(2.0f32);
    let MUL_3033 = MUL_3010 * Simd::splat(0.0793978f32);
    let MUL_2974 = ADD_853 * ADD_853;
    let ADD_2995 = MUL_2971 + MUL_2974;
    let MUL_2998 = ADD_2995 * Simd::splat(2.0f32);
    let SUB_3001 = Simd::splat(1.0f32) - MUL_2998;
    let MUL_3027 = SUB_3001 * Simd::splat(0.0133909f32);
    let ADD_3037 = MUL_3021 + MUL_3027;
    let ADD_3040 = ADD_3037 + MUL_3033;
    let MUL_821 = SUB_725 * MUL_806;
    let MUL_822 = ADD_708 * MUL_801;
    let ADD_824 = MUL_821 + MUL_822;
    let MUL_827 = ADD_824 * Simd::splat(2.0f32);
    let ADD_839 = ADD_565 + MUL_827;
    let ADD_3043 = ADD_839 + ADD_3040;
    let SUB_2989 = MUL_2977 - MUL_2973;
    let ADD_3002 = MUL_2978 + MUL_2975;
    let ADD_3011 = MUL_2970 + MUL_2974;
    let MUL_3014 = ADD_3011 * Simd::splat(2.0f32);
    let SUB_3017 = Simd::splat(1.0f32) - MUL_3014;
    let MUL_3035 = SUB_3017 * Simd::splat(0.0793978f32);
    let MUL_3004 = ADD_3002 * Simd::splat(2.0f32);
    let MUL_3029 = MUL_3004 * Simd::splat(0.0133909f32);
    let MUL_2991 = SUB_2989 * Simd::splat(2.0f32);
    let MUL_3023 = MUL_2991 * Simd::splat(0.0132698f32);
    let ADD_3038 = MUL_3023 + MUL_3029;
    let ADD_3041 = ADD_3038 + MUL_3035;
    let MUL_829 = SUB_725 * MUL_801;
    let MUL_831 = ADD_708 * MUL_806;
    let SUB_833 = MUL_831 - MUL_829;
    let MUL_836 = SUB_833 * Simd::splat(2.0f32);
    let ADD_840 = ADD_566 + MUL_836;
    let ADD_3044 = ADD_840 + ADD_3041;
    let MUL_3058 = MUL_3007 * Simd::splat(0.07f32);
    let ADD_3063 = ADD_838 + MUL_3058;
    let MUL_3060 = MUL_3010 * Simd::splat(0.07f32);
    let ADD_3064 = ADD_839 + MUL_3060;
    let MUL_3062 = SUB_3017 * Simd::splat(0.07f32);
    let ADD_3065 = ADD_840 + MUL_3062;
    let MUL_3079 = MUL_3007 * Simd::splat(0.08f32);
    let MUL_3067 = SUB_2985 * Simd::splat(0.02f32);
    let MUL_3073 = MUL_2994 * Simd::splat(0.04f32);
    let ADD_3084 = MUL_3067 + MUL_3073;
    let ADD_3087 = ADD_3084 + MUL_3079;
    let ADD_3090 = ADD_838 + ADD_3087;
    let MUL_3081 = MUL_3010 * Simd::splat(0.08f32);
    let MUL_3075 = SUB_3001 * Simd::splat(0.04f32);
    let MUL_3069 = MUL_2988 * Simd::splat(0.02f32);
    let ADD_3085 = MUL_3069 + MUL_3075;
    let ADD_3088 = ADD_3085 + MUL_3081;
    let ADD_3091 = ADD_839 + ADD_3088;
    let MUL_3083 = SUB_3017 * Simd::splat(0.08f32);
    let MUL_3077 = MUL_3004 * Simd::splat(0.04f32);
    let MUL_3071 = MUL_2991 * Simd::splat(0.02f32);
    let ADD_3086 = MUL_3071 + MUL_3077;
    let ADD_3089 = ADD_3086 + MUL_3083;
    let ADD_3092 = ADD_840 + ADD_3089;
    let MUL_3100 = MUL_2994 * Simd::splat(0.02f32);
    let MUL_3094 = SUB_2985 * Simd::splat(0.04f32);
    let ADD_3111 = MUL_3094 + MUL_3100;
    let ADD_3114 = ADD_3111 + MUL_3079;
    let ADD_3117 = ADD_838 + ADD_3114;
    let MUL_3102 = SUB_3001 * Simd::splat(0.02f32);
    let MUL_3096 = MUL_2988 * Simd::splat(0.04f32);
    let ADD_3112 = MUL_3096 + MUL_3102;
    let ADD_3115 = ADD_3112 + MUL_3081;
    let ADD_3118 = ADD_839 + ADD_3115;
    let MUL_3104 = MUL_3004 * Simd::splat(0.02f32);
    let MUL_3098 = MUL_2991 * Simd::splat(0.04f32);
    let ADD_3113 = MUL_3098 + MUL_3104;
    let ADD_3116 = ADD_3113 + MUL_3083;
    let ADD_3119 = ADD_840 + ADD_3116;
    let MUL_3133 = MUL_3007 * Simd::splat(0.085f32);
    let MUL_3127 = MUL_2994 * Simd::splat(0.06f32);
    let ADD_3138 = MUL_3094 + MUL_3127;
    let ADD_3141 = ADD_3138 + MUL_3133;
    let ADD_3144 = ADD_838 + ADD_3141;
    let MUL_3135 = MUL_3010 * Simd::splat(0.085f32);
    let MUL_3129 = SUB_3001 * Simd::splat(0.06f32);
    let ADD_3139 = MUL_3096 + MUL_3129;
    let ADD_3142 = ADD_3139 + MUL_3135;
    let ADD_3145 = ADD_839 + ADD_3142;
    let MUL_3137 = SUB_3017 * Simd::splat(0.085f32);
    let MUL_3131 = MUL_3004 * Simd::splat(0.06f32);
    let ADD_3140 = MUL_3098 + MUL_3131;
    let ADD_3143 = ADD_3140 + MUL_3137;
    let ADD_3146 = ADD_840 + ADD_3143;
    let MUL_3148 = SUB_2985 * Simd::splat(0.06f32);
    let ADD_3165 = MUL_3148 + MUL_3073;
    let ADD_3168 = ADD_3165 + MUL_3133;
    let ADD_3171 = ADD_838 + ADD_3168;
    let MUL_3150 = MUL_2988 * Simd::splat(0.06f32);
    let ADD_3166 = MUL_3150 + MUL_3075;
    let ADD_3169 = ADD_3166 + MUL_3135;
    let ADD_3172 = ADD_839 + ADD_3169;
    let MUL_3152 = MUL_2991 * Simd::splat(0.06f32);
    let ADD_3167 = MUL_3152 + MUL_3077;
    let ADD_3170 = ADD_3167 + MUL_3137;
    let ADD_3173 = ADD_840 + ADD_3170;
    if
    /* panda_link0 vs. panda_link7 */
    sphere_sphere_self_collision(
        Simd::splat(-0.043343f32),
        Simd::splat(1.4e-06f32),
        Simd::splat(0.0629063f32),
        Simd::splat(0.130366f32),
        ADD_3042,
        ADD_3043,
        ADD_3044,
        Simd::splat(0.073242f32),
    ) {
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
    } // (665, 821)
    if
    /* panda_link1 vs. panda_link7 */
    sphere_sphere_self_collision(
        ADD_1636,
        SUB_1637,
        Simd::splat(0.2598976f32),
        Simd::splat(0.144259f32),
        ADD_3042,
        ADD_3043,
        ADD_3044,
        Simd::splat(0.073242f32),
    ) {
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
    } // (821, 821)
    if
    /* panda_link2 vs. panda_link7 */
    sphere_sphere_self_collision(
        ADD_1835,
        SUB_1836,
        ADD_1838,
        Simd::splat(0.145067f32),
        ADD_3042,
        ADD_3043,
        ADD_3044,
        Simd::splat(0.073242f32),
    ) {
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
    } // (821, 821)
    if
    /* panda_link5 vs. panda_link7 */
    sphere_sphere_self_collision(
        ADD_2399,
        ADD_2400,
        ADD_2401,
        Simd::splat(0.173531f32),
        ADD_3042,
        ADD_3043,
        ADD_3044,
        Simd::splat(0.073242f32),
    ) {
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
    } // (821, 821)
    if
    /* panda_link7 */
    sphere_environment_in_collision(
        environment,
        ADD_3042,
        ADD_3043,
        ADD_3044,
        Simd::splat(0.073242f32),
    ) {
        if sphere_environment_in_collision(
            environment,
            ADD_3063,
            ADD_3064,
            ADD_3065,
            Simd::splat(0.05f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3090,
            ADD_3091,
            ADD_3092,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3117,
            ADD_3118,
            ADD_3119,
            Simd::splat(0.025f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3144,
            ADD_3145,
            ADD_3146,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3171,
            ADD_3172,
            ADD_3173,
            Simd::splat(0.02f32),
        ) {
            return false;
        }
    } // (821, 821)
    let MUL_1065 = SUB_870 * Simd::splat(0.9238795f32);
    let MUL_1062 = ADD_865 * Simd::splat(0.9238795f32);
    let MUL_1049 = SUB_859 * Simd::splat(0.9238795f32);
    let MUL_1034 = ADD_853 * Simd::splat(0.9238795f32);
    let MUL_1055 = SUB_870 * Simd::splat(0.3826834f32);
    let SUB_1063 = MUL_1062 - MUL_1055;
    let MUL_3235 = SUB_1063 * SUB_1063;
    let MUL_1072 = ADD_865 * Simd::splat(0.3826834f32);
    let ADD_1074 = MUL_1065 + MUL_1072;
    let MUL_3236 = ADD_1074 * SUB_1063;
    let MUL_1037 = SUB_859 * Simd::splat(0.3826834f32);
    let SUB_1039 = MUL_1034 - MUL_1037;
    let MUL_3241 = SUB_1039 * SUB_1063;
    let MUL_1046 = ADD_853 * Simd::splat(0.3826834f32);
    let ADD_1050 = MUL_1046 + MUL_1049;
    let MUL_3237 = ADD_1074 * ADD_1050;
    let ADD_3269 = MUL_3241 + MUL_3237;
    let MUL_3271 = ADD_3269 * Simd::splat(2.0f32);
    let MUL_3240 = SUB_1039 * ADD_1050;
    let SUB_3256 = MUL_3240 - MUL_3236;
    let MUL_3258 = SUB_3256 * Simd::splat(2.0f32);
    let MUL_3234 = ADD_1050 * ADD_1050;
    let ADD_3243 = MUL_3234 + MUL_3235;
    let MUL_3246 = ADD_3243 * Simd::splat(2.0f32);
    let SUB_3249 = Simd::splat(1.0f32) - MUL_3246;
    let MUL_931 = SUB_859 * Simd::splat(0.107f32);
    let MUL_942 = SUB_870 * MUL_931;
    let MUL_939 = ADD_853 * Simd::splat(0.107f32);
    let MUL_944 = ADD_865 * MUL_939;
    let ADD_945 = MUL_942 + MUL_944;
    let MUL_947 = ADD_945 * Simd::splat(2.0f32);
    let ADD_969 = ADD_838 + MUL_947;
    let MUL_3301 = MUL_3271 * Simd::splat(0.0196227f32);
    let MUL_3295 = MUL_3258 * Simd::splat(2.7e-05f32);
    let MUL_3284 = SUB_3249 * Simd::splat(5.1e-06f32);
    let SUB_3306 = MUL_3295 - MUL_3284;
    let ADD_3309 = SUB_3306 + MUL_3301;
    let ADD_3312 = ADD_969 + ADD_3309;
    let ADD_3250 = MUL_3240 + MUL_3236;
    let MUL_3252 = ADD_3250 * Simd::splat(2.0f32);
    let MUL_3288 = MUL_3252 * Simd::splat(5.1e-06f32);
    let MUL_3239 = ADD_1074 * SUB_1039;
    let MUL_3238 = SUB_1039 * SUB_1039;
    let ADD_3259 = MUL_3235 + MUL_3238;
    let MUL_3262 = ADD_3259 * Simd::splat(2.0f32);
    let SUB_3265 = Simd::splat(1.0f32) - MUL_3262;
    let MUL_3297 = SUB_3265 * Simd::splat(2.7e-05f32);
    let SUB_3307 = MUL_3297 - MUL_3288;
    let MUL_3242 = ADD_1050 * SUB_1063;
    let SUB_3272 = MUL_3242 - MUL_3239;
    let MUL_3274 = SUB_3272 * Simd::splat(2.0f32);
    let MUL_3303 = MUL_3274 * Simd::splat(0.0196227f32);
    let ADD_3310 = SUB_3307 + MUL_3303;
    let MUL_950 = SUB_870 * MUL_939;
    let MUL_953 = ADD_865 * MUL_931;
    let SUB_954 = MUL_953 - MUL_950;
    let MUL_956 = SUB_954 * Simd::splat(2.0f32);
    let ADD_970 = ADD_839 + MUL_956;
    let ADD_3313 = ADD_970 + ADD_3310;
    let SUB_3253 = MUL_3241 - MUL_3237;
    let ADD_3266 = MUL_3242 + MUL_3239;
    let ADD_3275 = MUL_3234 + MUL_3238;
    let MUL_3278 = ADD_3275 * Simd::splat(2.0f32);
    let SUB_3281 = Simd::splat(1.0f32) - MUL_3278;
    let MUL_3305 = SUB_3281 * Simd::splat(0.0196227f32);
    let MUL_3268 = ADD_3266 * Simd::splat(2.0f32);
    let MUL_3299 = MUL_3268 * Simd::splat(2.7e-05f32);
    let MUL_3255 = SUB_3253 * Simd::splat(2.0f32);
    let MUL_3292 = MUL_3255 * Simd::splat(5.1e-06f32);
    let SUB_3308 = MUL_3299 - MUL_3292;
    let ADD_3311 = SUB_3308 + MUL_3305;
    let MUL_961 = SUB_859 * MUL_931;
    let MUL_959 = ADD_853 * MUL_939;
    let ADD_962 = MUL_959 + MUL_961;
    let MUL_965 = ADD_962 * Simd::splat(2.0f32);
    let SUB_968 = Simd::splat(0.107f32) - MUL_965;
    let ADD_971 = ADD_840 + SUB_968;
    let ADD_3314 = ADD_971 + ADD_3311;
    let MUL_3334 = MUL_3271 * Simd::splat(0.01f32);
    let MUL_3323 = MUL_3258 * Simd::splat(0.075f32);
    let SUB_3339 = MUL_3334 - MUL_3323;
    let ADD_3342 = ADD_969 + SUB_3339;
    let MUL_3336 = MUL_3274 * Simd::splat(0.01f32);
    let MUL_3327 = SUB_3265 * Simd::splat(0.075f32);
    let SUB_3340 = MUL_3336 - MUL_3327;
    let ADD_3343 = ADD_970 + SUB_3340;
    let MUL_3338 = SUB_3281 * Simd::splat(0.01f32);
    let MUL_3331 = MUL_3268 * Simd::splat(0.075f32);
    let SUB_3341 = MUL_3338 - MUL_3331;
    let ADD_3344 = ADD_971 + SUB_3341;
    let MUL_3353 = MUL_3258 * Simd::splat(0.045f32);
    let SUB_3369 = MUL_3334 - MUL_3353;
    let ADD_3372 = ADD_969 + SUB_3369;
    let MUL_3357 = SUB_3265 * Simd::splat(0.045f32);
    let SUB_3370 = MUL_3336 - MUL_3357;
    let ADD_3373 = ADD_970 + SUB_3370;
    let MUL_3361 = MUL_3268 * Simd::splat(0.045f32);
    let SUB_3371 = MUL_3338 - MUL_3361;
    let ADD_3374 = ADD_971 + SUB_3371;
    let MUL_3383 = MUL_3258 * Simd::splat(0.015f32);
    let SUB_3399 = MUL_3334 - MUL_3383;
    let ADD_3402 = ADD_969 + SUB_3399;
    let MUL_3387 = SUB_3265 * Simd::splat(0.015f32);
    let SUB_3400 = MUL_3336 - MUL_3387;
    let ADD_3403 = ADD_970 + SUB_3400;
    let MUL_3391 = MUL_3268 * Simd::splat(0.015f32);
    let SUB_3401 = MUL_3338 - MUL_3391;
    let ADD_3404 = ADD_971 + SUB_3401;
    let ADD_3423 = MUL_3383 + MUL_3334;
    let ADD_3426 = ADD_969 + ADD_3423;
    let ADD_3424 = MUL_3387 + MUL_3336;
    let ADD_3427 = ADD_970 + ADD_3424;
    let ADD_3425 = MUL_3391 + MUL_3338;
    let ADD_3428 = ADD_971 + ADD_3425;
    let ADD_3447 = MUL_3353 + MUL_3334;
    let ADD_3450 = ADD_969 + ADD_3447;
    let ADD_3448 = MUL_3357 + MUL_3336;
    let ADD_3451 = ADD_970 + ADD_3448;
    let ADD_3449 = MUL_3361 + MUL_3338;
    let ADD_3452 = ADD_971 + ADD_3449;
    let ADD_3471 = MUL_3323 + MUL_3334;
    let ADD_3474 = ADD_969 + ADD_3471;
    let ADD_3472 = MUL_3327 + MUL_3336;
    let ADD_3475 = ADD_970 + ADD_3472;
    let ADD_3473 = MUL_3331 + MUL_3338;
    let ADD_3476 = ADD_971 + ADD_3473;
    let MUL_3496 = MUL_3271 * Simd::splat(0.03f32);
    let SUB_3501 = MUL_3496 - MUL_3323;
    let ADD_3504 = ADD_969 + SUB_3501;
    let MUL_3498 = MUL_3274 * Simd::splat(0.03f32);
    let SUB_3502 = MUL_3498 - MUL_3327;
    let ADD_3505 = ADD_970 + SUB_3502;
    let MUL_3500 = SUB_3281 * Simd::splat(0.03f32);
    let SUB_3503 = MUL_3500 - MUL_3331;
    let ADD_3506 = ADD_971 + SUB_3503;
    let SUB_3531 = MUL_3496 - MUL_3353;
    let ADD_3534 = ADD_969 + SUB_3531;
    let SUB_3532 = MUL_3498 - MUL_3357;
    let ADD_3535 = ADD_970 + SUB_3532;
    let SUB_3533 = MUL_3500 - MUL_3361;
    let ADD_3536 = ADD_971 + SUB_3533;
    let SUB_3561 = MUL_3496 - MUL_3383;
    let ADD_3564 = ADD_969 + SUB_3561;
    let SUB_3562 = MUL_3498 - MUL_3387;
    let ADD_3565 = ADD_970 + SUB_3562;
    let SUB_3563 = MUL_3500 - MUL_3391;
    let ADD_3566 = ADD_971 + SUB_3563;
    let ADD_3585 = MUL_3383 + MUL_3496;
    let ADD_3588 = ADD_969 + ADD_3585;
    let ADD_3586 = MUL_3387 + MUL_3498;
    let ADD_3589 = ADD_970 + ADD_3586;
    let ADD_3587 = MUL_3391 + MUL_3500;
    let ADD_3590 = ADD_971 + ADD_3587;
    let ADD_3609 = MUL_3353 + MUL_3496;
    let ADD_3612 = ADD_969 + ADD_3609;
    let ADD_3610 = MUL_3357 + MUL_3498;
    let ADD_3613 = ADD_970 + ADD_3610;
    let ADD_3611 = MUL_3361 + MUL_3500;
    let ADD_3614 = ADD_971 + ADD_3611;
    let ADD_3633 = MUL_3323 + MUL_3496;
    let ADD_3636 = ADD_969 + ADD_3633;
    let ADD_3634 = MUL_3327 + MUL_3498;
    let ADD_3637 = ADD_970 + ADD_3634;
    let ADD_3635 = MUL_3331 + MUL_3500;
    let ADD_3638 = ADD_971 + ADD_3635;
    let MUL_3658 = MUL_3271 * Simd::splat(0.05f32);
    let SUB_3663 = MUL_3658 - MUL_3323;
    let ADD_3666 = ADD_969 + SUB_3663;
    let MUL_3660 = MUL_3274 * Simd::splat(0.05f32);
    let SUB_3664 = MUL_3660 - MUL_3327;
    let ADD_3667 = ADD_970 + SUB_3664;
    let MUL_3662 = SUB_3281 * Simd::splat(0.05f32);
    let SUB_3665 = MUL_3662 - MUL_3331;
    let ADD_3668 = ADD_971 + SUB_3665;
    let SUB_3693 = MUL_3658 - MUL_3353;
    let ADD_3696 = ADD_969 + SUB_3693;
    let SUB_3694 = MUL_3660 - MUL_3357;
    let ADD_3697 = ADD_970 + SUB_3694;
    let SUB_3695 = MUL_3662 - MUL_3361;
    let ADD_3698 = ADD_971 + SUB_3695;
    let SUB_3723 = MUL_3658 - MUL_3383;
    let ADD_3726 = ADD_969 + SUB_3723;
    let SUB_3724 = MUL_3660 - MUL_3387;
    let ADD_3727 = ADD_970 + SUB_3724;
    let SUB_3725 = MUL_3662 - MUL_3391;
    let ADD_3728 = ADD_971 + SUB_3725;
    let ADD_3747 = MUL_3383 + MUL_3658;
    let ADD_3750 = ADD_969 + ADD_3747;
    let ADD_3748 = MUL_3387 + MUL_3660;
    let ADD_3751 = ADD_970 + ADD_3748;
    let ADD_3749 = MUL_3391 + MUL_3662;
    let ADD_3752 = ADD_971 + ADD_3749;
    let ADD_3771 = MUL_3353 + MUL_3658;
    let ADD_3774 = ADD_969 + ADD_3771;
    let ADD_3772 = MUL_3357 + MUL_3660;
    let ADD_3775 = ADD_970 + ADD_3772;
    let ADD_3773 = MUL_3361 + MUL_3662;
    let ADD_3776 = ADD_971 + ADD_3773;
    let ADD_3795 = MUL_3323 + MUL_3658;
    let ADD_3798 = ADD_969 + ADD_3795;
    let ADD_3796 = MUL_3327 + MUL_3660;
    let ADD_3799 = ADD_970 + ADD_3796;
    let ADD_3797 = MUL_3331 + MUL_3662;
    let ADD_3800 = ADD_971 + ADD_3797;
    if
    /* panda_hand */
    sphere_environment_in_collision(
        environment,
        ADD_3312,
        ADD_3313,
        ADD_3314,
        Simd::splat(0.107701f32),
    ) {
        if sphere_environment_in_collision(
            environment,
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
    } // (821, 1025)
    if
    /* panda_link0 vs. panda_hand */
    sphere_sphere_self_collision(
        Simd::splat(-0.043343f32),
        Simd::splat(1.4e-06f32),
        Simd::splat(0.0629063f32),
        Simd::splat(0.130366f32),
        ADD_3312,
        ADD_3313,
        ADD_3314,
        Simd::splat(0.107701f32),
    ) {
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
    } // (1025, 1025)
    if
    /* panda_link1 vs. panda_hand */
    sphere_sphere_self_collision(
        ADD_1636,
        SUB_1637,
        Simd::splat(0.2598976f32),
        Simd::splat(0.144259f32),
        ADD_3312,
        ADD_3313,
        ADD_3314,
        Simd::splat(0.107701f32),
    ) {
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
    } // (1025, 1025)
    if
    /* panda_link2 vs. panda_hand */
    sphere_sphere_self_collision(
        ADD_1835,
        SUB_1836,
        ADD_1838,
        Simd::splat(0.145067f32),
        ADD_3312,
        ADD_3313,
        ADD_3314,
        Simd::splat(0.107701f32),
    ) {
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
    } // (1025, 1025)
    if
    /* panda_link5 vs. panda_hand */
    sphere_sphere_self_collision(
        ADD_2399,
        ADD_2400,
        ADD_2401,
        Simd::splat(0.173531f32),
        ADD_3312,
        ADD_3313,
        ADD_3314,
        Simd::splat(0.107701f32),
    ) {
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3342,
            ADD_3343,
            ADD_3344,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3372,
            ADD_3373,
            ADD_3374,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3402,
            ADD_3403,
            ADD_3404,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3426,
            ADD_3427,
            ADD_3428,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3450,
            ADD_3451,
            ADD_3452,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3474,
            ADD_3475,
            ADD_3476,
            Simd::splat(0.028f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3504,
            ADD_3505,
            ADD_3506,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3534,
            ADD_3535,
            ADD_3536,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3564,
            ADD_3565,
            ADD_3566,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3588,
            ADD_3589,
            ADD_3590,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3612,
            ADD_3613,
            ADD_3614,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3636,
            ADD_3637,
            ADD_3638,
            Simd::splat(0.026f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3666,
            ADD_3667,
            ADD_3668,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3696,
            ADD_3697,
            ADD_3698,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3726,
            ADD_3727,
            ADD_3728,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3750,
            ADD_3751,
            ADD_3752,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3774,
            ADD_3775,
            ADD_3776,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3798,
            ADD_3799,
            ADD_3800,
            Simd::splat(0.024f32),
        ) {
            return false;
        }
    } // (1025, 1025)
    let MUL_3876 = ADD_3269 * Simd::splat(2.0f32);
    let MUL_3863 = SUB_3256 * Simd::splat(2.0f32);
    let MUL_3851 = ADD_3243 * Simd::splat(2.0f32);
    let SUB_3854 = Simd::splat(1.0f32) - MUL_3851;
    let MUL_1196 = SUB_1063 * Simd::splat(0.065f32);
    let MUL_1199 = SUB_1039 * Simd::splat(0.065f32);
    let MUL_1207 = ADD_1050 * MUL_1199;
    let MUL_1203 = SUB_1039 * Simd::splat(0.0584f32);
    let MUL_1209 = SUB_1063 * MUL_1203;
    let MUL_1194 = ADD_1050 * Simd::splat(0.0584f32);
    let SUB_1197 = MUL_1194 - MUL_1196;
    let MUL_1206 = ADD_1074 * SUB_1197;
    let ADD_1208 = MUL_1206 + MUL_1207;
    let ADD_1210 = ADD_1208 + MUL_1209;
    let MUL_1212 = ADD_1210 * Simd::splat(2.0f32);
    let ADD_1235 = ADD_969 + MUL_1212;
    let MUL_3906 = MUL_3876 * Simd::splat(0.0261043f32);
    let MUL_3900 = MUL_3863 * Simd::splat(0.0129294f32);
    let MUL_3889 = SUB_3854 * Simd::splat(4.3e-06f32);
    let SUB_3911 = MUL_3900 - MUL_3889;
    let ADD_3914 = SUB_3911 + MUL_3906;
    let ADD_3917 = ADD_1235 + ADD_3914;
    let MUL_3879 = SUB_3272 * Simd::splat(2.0f32);
    let MUL_3908 = MUL_3879 * Simd::splat(0.0261043f32);
    let MUL_3867 = ADD_3259 * Simd::splat(2.0f32);
    let SUB_3870 = Simd::splat(1.0f32) - MUL_3867;
    let MUL_3902 = SUB_3870 * Simd::splat(0.0129294f32);
    let MUL_3857 = ADD_3250 * Simd::splat(2.0f32);
    let MUL_3893 = MUL_3857 * Simd::splat(4.3e-06f32);
    let SUB_3912 = MUL_3902 - MUL_3893;
    let ADD_3915 = SUB_3912 + MUL_3908;
    let MUL_1215 = ADD_1074 * MUL_1203;
    let MUL_1220 = SUB_1063 * SUB_1197;
    let MUL_1217 = SUB_1039 * MUL_1199;
    let ADD_1218 = MUL_1215 + MUL_1217;
    let SUB_1221 = MUL_1220 - ADD_1218;
    let MUL_1223 = SUB_1221 * Simd::splat(2.0f32);
    let ADD_1225 = MUL_1223 + Simd::splat(0.065f32);
    let ADD_1236 = ADD_970 + ADD_1225;
    let ADD_3918 = ADD_1236 + ADD_3915;
    let MUL_3883 = ADD_3275 * Simd::splat(2.0f32);
    let SUB_3886 = Simd::splat(1.0f32) - MUL_3883;
    let MUL_3910 = SUB_3886 * Simd::splat(0.0261043f32);
    let MUL_3873 = ADD_3266 * Simd::splat(2.0f32);
    let MUL_3904 = MUL_3873 * Simd::splat(0.0129294f32);
    let MUL_3860 = SUB_3253 * Simd::splat(2.0f32);
    let MUL_3897 = MUL_3860 * Simd::splat(4.3e-06f32);
    let SUB_3913 = MUL_3904 - MUL_3897;
    let ADD_3916 = SUB_3913 + MUL_3910;
    let MUL_1226 = ADD_1074 * MUL_1199;
    let MUL_1227 = SUB_1039 * MUL_1203;
    let SUB_1228 = MUL_1226 - MUL_1227;
    let MUL_1229 = ADD_1050 * SUB_1197;
    let SUB_1230 = SUB_1228 - MUL_1229;
    let MUL_1232 = SUB_1230 * Simd::splat(2.0f32);
    let ADD_1234 = MUL_1232 + Simd::splat(0.0584f32);
    let ADD_1237 = ADD_971 + ADD_1234;
    let ADD_3919 = ADD_1237 + ADD_3916;
    let MUL_3927 = MUL_3863 * Simd::splat(0.015f32);
    let MUL_3933 = MUL_3876 * Simd::splat(0.022f32);
    let ADD_3938 = MUL_3927 + MUL_3933;
    let ADD_3941 = ADD_1235 + ADD_3938;
    let MUL_3935 = MUL_3879 * Simd::splat(0.022f32);
    let MUL_3929 = SUB_3870 * Simd::splat(0.015f32);
    let ADD_3939 = MUL_3929 + MUL_3935;
    let ADD_3942 = ADD_1236 + ADD_3939;
    let MUL_3937 = SUB_3886 * Simd::splat(0.022f32);
    let MUL_3931 = MUL_3873 * Simd::splat(0.015f32);
    let ADD_3940 = MUL_3931 + MUL_3937;
    let ADD_3943 = ADD_1237 + ADD_3940;
    let MUL_3957 = MUL_3876 * Simd::splat(0.044f32);
    let MUL_3951 = MUL_3863 * Simd::splat(0.008f32);
    let ADD_3962 = MUL_3951 + MUL_3957;
    let ADD_3965 = ADD_1235 + ADD_3962;
    let MUL_3959 = MUL_3879 * Simd::splat(0.044f32);
    let MUL_3953 = SUB_3870 * Simd::splat(0.008f32);
    let ADD_3963 = MUL_3953 + MUL_3959;
    let ADD_3966 = ADD_1236 + ADD_3963;
    let MUL_3961 = SUB_3886 * Simd::splat(0.044f32);
    let MUL_3955 = MUL_3873 * Simd::splat(0.008f32);
    let ADD_3964 = MUL_3955 + MUL_3961;
    let ADD_3967 = ADD_1237 + ADD_3964;
    if
    /* panda_leftfinger */
    sphere_environment_in_collision(
        environment,
        ADD_3917,
        ADD_3918,
        ADD_3919,
        Simd::splat(0.031022f32),
    ) {
        if sphere_environment_in_collision(
            environment,
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
    } // (1025, 1107)
    if
    /* panda_link0 vs. panda_leftfinger */
    sphere_sphere_self_collision(
        Simd::splat(-0.043343f32),
        Simd::splat(1.4e-06f32),
        Simd::splat(0.0629063f32),
        Simd::splat(0.130366f32),
        ADD_3917,
        ADD_3918,
        ADD_3919,
        Simd::splat(0.031022f32),
    ) {
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
    } // (1107, 1107)
    if
    /* panda_link1 vs. panda_leftfinger */
    sphere_sphere_self_collision(
        ADD_1636,
        SUB_1637,
        Simd::splat(0.2598976f32),
        Simd::splat(0.144259f32),
        ADD_3917,
        ADD_3918,
        ADD_3919,
        Simd::splat(0.031022f32),
    ) {
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
    } // (1107, 1107)
    if
    /* panda_link2 vs. panda_leftfinger */
    sphere_sphere_self_collision(
        ADD_1835,
        SUB_1836,
        ADD_1838,
        Simd::splat(0.145067f32),
        ADD_3917,
        ADD_3918,
        ADD_3919,
        Simd::splat(0.031022f32),
    ) {
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
    } // (1107, 1107)
    if
    /* panda_link5 vs. panda_leftfinger */
    sphere_sphere_self_collision(
        ADD_2399,
        ADD_2400,
        ADD_2401,
        Simd::splat(0.173531f32),
        ADD_3917,
        ADD_3918,
        ADD_3919,
        Simd::splat(0.031022f32),
    ) {
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3941,
            ADD_3942,
            ADD_3943,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_3965,
            ADD_3966,
            ADD_3967,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
    } // (1107, 1107)
    let ADD_1331 = MUL_1194 + MUL_1196;
    let MUL_4011 = ADD_3269 * Simd::splat(2.0f32);
    let MUL_3998 = SUB_3256 * Simd::splat(2.0f32);
    let MUL_3986 = ADD_3243 * Simd::splat(2.0f32);
    let SUB_3989 = Simd::splat(1.0f32) - MUL_3986;
    let MUL_1342 = ADD_1074 * ADD_1331;
    let SUB_1345 = MUL_1342 - MUL_1207;
    let ADD_1347 = SUB_1345 + MUL_1209;
    let MUL_1349 = ADD_1347 * Simd::splat(2.0f32);
    let ADD_1377 = ADD_969 + MUL_1349;
    let MUL_4041 = MUL_4011 * Simd::splat(0.0261203f32);
    let MUL_4030 = MUL_3998 * Simd::splat(0.0129624f32);
    let MUL_4023 = SUB_3989 * Simd::splat(5.4e-06f32);
    let SUB_4046 = MUL_4023 - MUL_4030;
    let ADD_4049 = SUB_4046 + MUL_4041;
    let ADD_4052 = ADD_1377 + ADD_4049;
    let SUB_1356 = MUL_1217 - MUL_1215;
    let MUL_4014 = SUB_3272 * Simd::splat(2.0f32);
    let MUL_4043 = MUL_4014 * Simd::splat(0.0261203f32);
    let MUL_4002 = ADD_3259 * Simd::splat(2.0f32);
    let SUB_4005 = Simd::splat(1.0f32) - MUL_4002;
    let MUL_4034 = SUB_4005 * Simd::splat(0.0129624f32);
    let MUL_3992 = ADD_3250 * Simd::splat(2.0f32);
    let MUL_4025 = MUL_3992 * Simd::splat(5.4e-06f32);
    let SUB_4047 = MUL_4025 - MUL_4034;
    let ADD_4050 = SUB_4047 + MUL_4043;
    let MUL_1357 = SUB_1063 * ADD_1331;
    let ADD_1358 = SUB_1356 + MUL_1357;
    let MUL_1360 = ADD_1358 * Simd::splat(2.0f32);
    let SUB_1363 = MUL_1360 - Simd::splat(0.065f32);
    let ADD_1378 = ADD_970 + SUB_1363;
    let ADD_4053 = ADD_1378 + ADD_4050;
    let ADD_1367 = MUL_1226 + MUL_1227;
    let MUL_4018 = ADD_3275 * Simd::splat(2.0f32);
    let SUB_4021 = Simd::splat(1.0f32) - MUL_4018;
    let MUL_4045 = SUB_4021 * Simd::splat(0.0261203f32);
    let MUL_4008 = ADD_3266 * Simd::splat(2.0f32);
    let MUL_4038 = MUL_4008 * Simd::splat(0.0129624f32);
    let MUL_3995 = SUB_3253 * Simd::splat(2.0f32);
    let MUL_4027 = MUL_3995 * Simd::splat(5.4e-06f32);
    let SUB_4048 = MUL_4027 - MUL_4038;
    let ADD_4051 = SUB_4048 + MUL_4045;
    let MUL_1369 = ADD_1050 * ADD_1331;
    let ADD_1370 = ADD_1367 + MUL_1369;
    let MUL_1373 = ADD_1370 * Simd::splat(2.0f32);
    let SUB_1376 = Simd::splat(0.0584f32) - MUL_1373;
    let ADD_1379 = ADD_971 + SUB_1376;
    let ADD_4054 = ADD_1379 + ADD_4051;
    let MUL_4074 = MUL_4011 * Simd::splat(0.022f32);
    let MUL_4063 = MUL_3998 * Simd::splat(0.015f32);
    let SUB_4079 = MUL_4074 - MUL_4063;
    let ADD_4082 = ADD_1377 + SUB_4079;
    let MUL_4076 = MUL_4014 * Simd::splat(0.022f32);
    let MUL_4067 = SUB_4005 * Simd::splat(0.015f32);
    let SUB_4080 = MUL_4076 - MUL_4067;
    let ADD_4083 = ADD_1378 + SUB_4080;
    let MUL_4078 = SUB_4021 * Simd::splat(0.022f32);
    let MUL_4071 = MUL_4008 * Simd::splat(0.015f32);
    let SUB_4081 = MUL_4078 - MUL_4071;
    let ADD_4084 = ADD_1379 + SUB_4081;
    let MUL_4104 = MUL_4011 * Simd::splat(0.044f32);
    let MUL_4093 = MUL_3998 * Simd::splat(0.008f32);
    let SUB_4109 = MUL_4104 - MUL_4093;
    let ADD_4112 = ADD_1377 + SUB_4109;
    let MUL_4106 = MUL_4014 * Simd::splat(0.044f32);
    let MUL_4097 = SUB_4005 * Simd::splat(0.008f32);
    let SUB_4110 = MUL_4106 - MUL_4097;
    let ADD_4113 = ADD_1378 + SUB_4110;
    let MUL_4108 = SUB_4021 * Simd::splat(0.044f32);
    let MUL_4101 = MUL_4008 * Simd::splat(0.008f32);
    let SUB_4111 = MUL_4108 - MUL_4101;
    let ADD_4114 = ADD_1379 + SUB_4111;
    if
    /* panda_link0 vs. panda_rightfinger */
    sphere_sphere_self_collision(
        Simd::splat(-0.043343f32),
        Simd::splat(1.4e-06f32),
        Simd::splat(0.0629063f32),
        Simd::splat(0.130366f32),
        ADD_4052,
        ADD_4053,
        ADD_4054,
        Simd::splat(0.031022f32),
    ) {
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.05f32),
            Simd::splat(0.08f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
    } // (1107, 1179)
    if
    /* panda_link1 vs. panda_rightfinger */
    sphere_sphere_self_collision(
        ADD_1636,
        SUB_1637,
        Simd::splat(0.2598976f32),
        Simd::splat(0.144259f32),
        ADD_4052,
        ADD_4053,
        ADD_4054,
        Simd::splat(0.031022f32),
    ) {
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1650,
            NEGATE_1654,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1674,
            NEGATE_1678,
            Simd::splat(0.333f32),
            Simd::splat(0.06f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.213f32),
            Simd::splat(0.06f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            Simd::splat(0.0f32),
            Simd::splat(0.0f32),
            Simd::splat(0.163f32),
            Simd::splat(0.06f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
    } // (1179, 1179)
    if
    /* panda_link2 vs. panda_rightfinger */
    sphere_sphere_self_collision(
        ADD_1835,
        SUB_1836,
        ADD_1838,
        Simd::splat(0.145067f32),
        ADD_4052,
        ADD_4053,
        ADD_4054,
        Simd::splat(0.031022f32),
    ) {
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1852,
            MUL_1854,
            ADD_1857,
            Simd::splat(0.06f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1871,
            MUL_1873,
            ADD_1876,
            Simd::splat(0.06f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1885,
            NEGATE_1889,
            SUB_1900,
            Simd::splat(0.06f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            MUL_1909,
            NEGATE_1913,
            SUB_1924,
            Simd::splat(0.06f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
    } // (1179, 1179)
    if
    /* panda_link5 vs. panda_rightfinger */
    sphere_sphere_self_collision(
        ADD_2399,
        ADD_2400,
        ADD_2401,
        Simd::splat(0.173531f32),
        ADD_4052,
        ADD_4053,
        ADD_4054,
        Simd::splat(0.031022f32),
    ) {
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2420,
            ADD_2421,
            ADD_2422,
            Simd::splat(0.06f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2441,
            ADD_2442,
            ADD_2443,
            Simd::splat(0.06f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            SUB_2468,
            SUB_2469,
            SUB_2470,
            Simd::splat(0.06f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2498,
            ADD_2499,
            ADD_2500,
            Simd::splat(0.05f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2531,
            ADD_2532,
            ADD_2533,
            Simd::splat(0.025f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2564,
            ADD_2565,
            ADD_2566,
            Simd::splat(0.025f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2597,
            ADD_2598,
            ADD_2599,
            Simd::splat(0.025f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2630,
            ADD_2631,
            ADD_2632,
            Simd::splat(0.025f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2669,
            ADD_2670,
            ADD_2671,
            Simd::splat(0.025f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2708,
            ADD_2709,
            ADD_2710,
            Simd::splat(0.025f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2747,
            ADD_2748,
            ADD_2749,
            Simd::splat(0.025f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_sphere_self_collision(
            ADD_2786,
            ADD_2787,
            ADD_2788,
            Simd::splat(0.025f32),
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
    } // (1179, 1179)
    if
    /* panda_rightfinger */
    sphere_environment_in_collision(
        environment,
        ADD_4052,
        ADD_4053,
        ADD_4054,
        Simd::splat(0.031022f32),
    ) {
        if sphere_environment_in_collision(
            environment,
            ADD_4082,
            ADD_4083,
            ADD_4084,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
        if sphere_environment_in_collision(
            environment,
            ADD_4112,
            ADD_4113,
            ADD_4114,
            Simd::splat(0.012f32),
        ) {
            return false;
        }
    } // (1179, 1179)
    true
}
