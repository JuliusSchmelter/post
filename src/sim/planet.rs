/*
 * Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 17.11.23
 * Last modified by Tibor Völcker on 18.11.23
 * Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)
 */
use nalgebra::Vector3;

pub trait Planet {
    fn gravity(&self, position: Vector3<f32>) -> Vector3<f32>;
}
