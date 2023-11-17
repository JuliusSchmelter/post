/*
 * Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
 * Last modified by Tibor Völcker on 17.11.23
 * Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)
 */
use nalgebra::SVector;

mod integration;

pub trait System<const D: usize> {
    fn system(&self, time: f32, state: &SVector<f32, D>) -> SVector<f32, D>;

    fn get_state(&self) -> &SVector<f32, D>;

    fn get_time(&self) -> f32;

    fn set_state(&mut self, state: SVector<f32, D>);

    fn set_time(&mut self, time: f32);
}
