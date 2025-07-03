use serde::ser::{Serialize, Serializer, SerializeTupleStruct};
use serde::de::{self, Deserialize, Deserializer, Visitor, SeqAccess};
use std::fmt;

use crate::{Vec2xz, DVec2xz};

macro_rules! impl_serde_vec2 {
    ($t:ty, $vec2:ident) => {
        /// Serialize as a sequence of 2 values.
        impl Serialize for $vec2 {
            fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
            where
                S: Serializer,
            {
                let mut state = serializer.serialize_tuple_struct(stringify!($vec2), 2)?;
                state.serialize_field(&self.x)?;
                state.serialize_field(&self.z)?;
                state.end()
            }
        }

        /// Deserialize expects a sequence of 2 values.
        impl<'de> Deserialize<'de> for $vec2 {
            fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
            where
                D: Deserializer<'de>,
            {
                struct Vec2Visitor;

                impl<'de> Visitor<'de> for Vec2Visitor {
                    type Value = $vec2;

                    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                        formatter.write_str(&concat!("a sequence of 2 ", stringify!($t), " values"))
                    }

                    fn visit_seq<V>(self, mut seq: V) -> Result<$vec2, V::Error>
                    where
                        V: SeqAccess<'de>,
                    {
                        let x = seq
                            .next_element()?
                            .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                        let y = seq
                            .next_element()?
                            .ok_or_else(|| de::Error::invalid_length(1, &self))?;
                        Ok($vec2::new(x, y))
                    }
                }

                deserializer.deserialize_tuple_struct(stringify!($vec2), 2, Vec2Visitor)
            }
        }
    };
}

impl_serde_vec2!(f32, Vec2xz);
impl_serde_vec2!(f64, DVec2xz);
