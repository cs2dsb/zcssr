extern crate proc_macro;
use crate::proc_macro::TokenStream;

use proc_macro2::{Ident, Span};
use quote::quote;
use syn;

/// Converts a string literal into a GenericArray<AsciiChar, Uxx> where xx is the
/// number of characters in the string
///
/// Uses a union to transmute [AsciiChar; xx] into the GenericArray. There is a macro
/// called `arr` in GenericArray that does something similar but it uses core::mem::transmute
/// which is not currently a const fn. See https://github.com/rust-lang/rust/issues/53605 for
/// progress on making transmute const.
///
/// # Example
/// ```
/// const HELLO: GenericArray<AsciiChar, U5> = str_to_generic_array_of_ascii_char!("HELLO");
/// ```
#[proc_macro]
pub fn str_to_generic_array_of_ascii_char(input: TokenStream) -> TokenStream {
    let ast = syn::parse(input).unwrap();

    impl_str_to_generic_array_of_ascii_char(&ast)
}

fn impl_str_to_generic_array_of_ascii_char(input: &syn::LitStr) -> TokenStream {
    let string = input.value();
    let count = string.len();
    let chars = string.chars();
    let typenum = format!("U{}", count);
    let typenum_ident = Ident::new(&typenum, Span::call_site());

    let result = quote! {
        {
            union Transmute {
                from: [ascii::AsciiChar; #count],
                to: generic_array::GenericArray<ascii::AsciiChar, typenum::#typenum_ident>,
            };
            unsafe { Transmute { from: [#(ascii::AsciiChar::new(#chars)),*] }.to }
        }
    };
    result.into()
}