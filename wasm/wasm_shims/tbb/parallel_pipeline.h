#pragma once

#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>

namespace oneapi { namespace tbb {

enum class filter_mode {
    serial_in_order,
    serial_out_of_order,
    parallel
};

class filter {
public:
    enum mode {
        serial_in_order,
        serial_out_of_order,
        parallel
    };
};

struct flow_control {
    void stop() { m_stop = true; }
    bool is_stopped() const { return m_stop; }

private:
    bool m_stop = false;
};

namespace detail {

template <typename Input, typename Output, typename Func>
struct filter_wrapper {
    using input_type = Input;
    using output_type = Output;

    filter_mode mode;
    Func func;

    Output operator()(Input value) const {
        return func(std::move(value));
    }
};

template <typename Output, typename Func>
struct filter_wrapper<void, Output, Func> {
    using input_type = void;
    using output_type = Output;

    filter_mode mode;
    Func func;

    Output operator()(flow_control& fc) const {
        return func(fc);
    }
};

template <typename Input, typename Func>
struct filter_wrapper<Input, void, Func> {
    using input_type = Input;
    using output_type = void;

    filter_mode mode;
    Func func;

    void operator()(Input value) const {
        func(std::move(value));
    }
};

template <typename Func>
struct filter_wrapper<void, void, Func> {
    using input_type = void;
    using output_type = void;

    filter_mode mode;
    Func func;

    void operator()(flow_control& fc) const {
        func(fc);
    }
};

template <typename Left, typename Right>
struct filter_sequence {
    Left left;
    Right right;
};

template <typename Pipeline>
auto flatten_pipeline(const Pipeline& pipeline) {
    return std::tuple<Pipeline>{pipeline};
}

template <typename Left, typename Right>
auto flatten_pipeline(const filter_sequence<Left, Right>& seq) {
    return std::tuple_cat(flatten_pipeline(seq.left), flatten_pipeline(seq.right));
}

template <typename Tuple, std::size_t... Indices>
auto forward_as_tuple_impl(Tuple&& tuple, std::index_sequence<Indices...>) {
    return std::tuple<std::decay_t<std::tuple_element_t<Indices, Tuple>>...>(
        std::get<Indices>(std::forward<Tuple>(tuple))...);
}

template <typename Tuple>
auto forward_as_tuple(Tuple&& tuple) {
    constexpr std::size_t size = std::tuple_size_v<std::decay_t<Tuple>>;
    return forward_as_tuple_impl(std::forward<Tuple>(tuple), std::make_index_sequence<size>{});
}

template <std::size_t Index, typename Tuple, typename Value>
void propagate(flow_control& fc, const Tuple& filters, Value&& value) {
    if constexpr (Index >= std::tuple_size_v<Tuple>) {
        (void)fc;
        (void)filters;
        (void)value;
    } else {
        const auto& filter = std::get<Index>(filters);
        using filter_t = std::decay_t<decltype(filter)>;

        if constexpr (std::is_void_v<typename filter_t::input_type>) {
            auto next_value = filter(fc);
            propagate<Index + 1>(fc, filters, std::move(next_value));
        } else if constexpr (std::is_void_v<typename filter_t::output_type>) {
            filter(std::forward<Value>(value));
            propagate<Index + 1>(fc, filters);
        } else {
            auto next_value = filter(std::forward<Value>(value));
            propagate<Index + 1>(fc, filters, std::move(next_value));
        }
    }
}

template <std::size_t Index, typename Tuple>
void propagate(flow_control& fc, const Tuple& filters) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        const auto& filter = std::get<Index>(filters);
        using filter_t = std::decay_t<decltype(filter)>;
        static_assert(std::is_void_v<typename filter_t::input_type>,
                      "Filter expects an input value that is not provided");
        auto value = filter(fc);
        propagate<Index + 1>(fc, filters, std::move(value));
    } else {
        (void)fc;
        (void)filters;
    }
}

template <typename Tuple>
void run_pipeline(flow_control& fc, const Tuple& filters) {
    if constexpr (std::tuple_size_v<Tuple> == 0) {
        (void)fc;
        (void)filters;
    } else {
        using first_filter_t = std::decay_t<std::tuple_element_t<0, Tuple>>;
        while (!fc.is_stopped()) {
            auto value = std::get<0>(filters)(fc);
            // TBB semantics: once the input filter calls fc.stop(), the value it
            // returned is a dummy and must NOT flow downstream.
            if (fc.is_stopped()) break;
            if constexpr (std::tuple_size_v<Tuple> > 1) {
                if constexpr (std::is_void_v<typename first_filter_t::output_type>) {
                    propagate<1>(fc, filters);
                } else {
                    propagate<1>(fc, filters, std::move(value));
                }
            } else {
                (void)value;
            }
        }
    }
}


// Pipeline composition. Declared inside `detail` so argument-dependent lookup
// finds them for filter_wrapper / filter_sequence operands (the old generic
// operator& in oneapi::tbb was invisible to ADL and matched any type).
template <typename T> struct is_filter_part : std::false_type {};
template <typename I, typename O, typename F> struct is_filter_part<filter_wrapper<I, O, F>> : std::true_type {};
template <typename L, typename R> struct is_filter_part<filter_sequence<L, R>> : std::true_type {};

template <typename Left, typename Right,
          typename = std::enable_if_t<is_filter_part<std::decay_t<Left>>::value && is_filter_part<std::decay_t<Right>>::value>>
auto operator&(Left&& left, Right&& right) {
    return filter_sequence<std::decay_t<Left>, std::decay_t<Right>>{
        std::forward<Left>(left), std::forward<Right>(right)};
}

} // namespace detail

template <typename Input, typename Output, typename Func>
auto make_filter(filter_mode mode, Func&& func) {
    return detail::filter_wrapper<Input, Output, std::decay_t<Func>>{mode, std::forward<Func>(func)};
}

// One entry point taking the pipeline by const reference. (The previous shim
// also had a `Pipeline&&` forwarding overload that copied into an lvalue and
// called itself again -> unbounded recursion -> stack overflow, which in the
// browser surfaces as "memory access out of bounds".)
template <typename Pipeline>
void parallel_pipeline(std::size_t /*max_number_of_live_tokens*/, const Pipeline& pipeline) {
    auto filters = detail::flatten_pipeline(pipeline);
    flow_control fc;
    detail::run_pipeline(fc, filters);
}

}} // namespace oneapi::tbb

#ifndef ORCA_WASM_TBB_ALIAS_DEFINED
#define ORCA_WASM_TBB_ALIAS_DEFINED
namespace tbb = oneapi::tbb;
#endif
