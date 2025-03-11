#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::convert::From;
use std::num::NonZero;
use std::ops::{Add, Div, Range};

// TODO: Implement a version that takes references to the underlying object
// TODO: Implement a version that takes points and id:s
// TODO: Implement a version that incrementally updates the tree when
//       points move

#[derive(Debug)]
pub struct KdTree<'a, T, const K: usize>
where
    //&'a Item: Into<[T; K]>,
    [(); 1 << K]: Sized,
{
    pub root: Node<'a, T, K>,
    pub count: usize,
    #[allow(dead_code)]
    capacity: NonZero<usize>,
}

#[derive(Debug)]
pub enum Node<'a, T, const K: usize>
where
    //&'a Item: Into<[T; K]>,
    [(); 1 << K]: Sized,
{
    Leaf {
        ranges: [Range<T>; K],
        points: Vec<[T; K]>,
        ids: Vec<usize>,
        capacity: NonZero<usize>,
    },
    Parent {
        ranges: [Range<T>; K],
        children: Box<[Node<'a, T, K>; 1 << K]>,
        #[allow(dead_code)]
        capacity: NonZero<usize>,
    },
}

#[derive(Debug)]
pub enum ErrorKind {
    EmptyRegion,
    NodeAlreadySplit,
    OutOfBounds,
}

pub trait AsCoordinates<T, const K: usize> {
    fn coordinates(self) -> [T; K];
}
impl<S, T, const K: usize> AsCoordinates<T, K> for S
where
    T: Copy,
    S: Into<[T; K]> + ?Sized,
{
    fn coordinates(self) -> [T; K] {
        self.into()
    }
}

/*
impl<T, const K: usize> AsCoordinates<T, K> for [T; K]
where
    T: Copy,
{
    fn coordinates(&self) -> [T; K] {
        *self
    }
}
*/

pub trait Two<T> {
    fn two() -> T;
}

impl<T> Two<T> for T
where
    T: From<f32>,
{
    #[inline]
    fn two() -> T {
        2.0.into()
    }
}

pub trait Halveable:
    Sized + PartialOrd + Add<Output = Self> + Div<Output = Self> + Two<Self>
{
}

impl<T> Halveable for T where
    T: Sized + PartialOrd + Add<Output = Self> + Div<Output = Self> + Two<Self>
{
}

pub trait MidPoint<T> {
    fn mid_point(&self) -> T;
}

impl<T> MidPoint<T> for Range<T>
where
    T: Halveable + Clone,
{
    fn mid_point(&self) -> T {
        (self.end.clone() + self.start.clone()) / T::two()
    }
}

impl<'a, T, const K: usize> Node<'a, T, K>
where
    T: Halveable + Clone,
    Range<T>: MidPoint<T>,
    [(); 1 << K]: Sized,
{
    fn new(ranges: [Range<T>; K], capacity: NonZero<usize>) -> Self {
        Self::Leaf {
            ranges,
            capacity,
            points: vec![],
            ids: vec![],
        }
    }

    fn insert(&mut self, point: [T; K], id: usize) {
        match &mut *self {
            Self::Leaf {
                points,
                ids,
                capacity,
                ..
            } => {
                points.push(point);
                ids.push(id);
                if points.len() > capacity.get() {
                    let owned_points = std::mem::take(points);
                    let owned_ids = std::mem::take(ids);
                    let _ = self.split();
                    owned_points
                        .into_iter()
                        .zip(owned_ids)
                        .for_each(|(p, id)| self.insert(p, id));
                }
            }
            Self::Parent {
                ranges, children, ..
            } => {
                let mut idx: usize = 0;
                ranges.iter().enumerate().for_each(|(axis, range)| {
                    if point[axis] >= range.mid_point() {
                        idx += 1 << axis;
                    }
                });
                children[idx].insert(point, id);
            }
        };
    }

    #[inline]
    fn get_ranges(&self) -> &[Range<T>; K] {
        match self {
            Self::Leaf { ranges, .. } => ranges,
            Self::Parent { ranges, .. } => ranges,
        }
    }

    // If [a, b) is the parent node's range along a certain axis,
    // then the child's range along that axis will be either
    // [a, c) or [c, b), where c = (b-a) / 2 is the midpoint.
    // As such, since there are K axes, a split will result in
    // 2^K distinct children. We can encode the unique
    // configuration of a particular child by using the 2^K
    // binary numbers from 0 to 2^K - 1. If the i:th bit is 1,
    // then the range along the i:th axis is [c, b), otherwise
    // the range is [a, c).
    fn split(&mut self) -> Result<(), ErrorKind> {
        // If the range has 0 volume, it makes no sense to split.
        if self.get_ranges().iter().any(|range| range.is_empty()) {
            return Err(ErrorKind::EmptyRegion);
        }
        let self_owned = unsafe { std::ptr::read(self) };
        match self_owned {
            // Splitting path
            Self::Leaf {
                ranges, capacity, ..
            } => {
                let children: [Node<T, K>; 1 << K] =
                    std::array::from_fn(|combination| {
                        let subranges: [Range<T>; K] =
                            std::array::from_fn(|i| {
                                let range = &ranges[i];
                                let mid_point = range.mid_point();
                                if combination & (1 << i) == 0 {
                                    range.start.clone()..mid_point
                                } else {
                                    mid_point..range.end.clone()
                                }
                            });
                        Node::new(subranges, capacity)
                    });

                let new_self = Self::Parent {
                    ranges,
                    capacity,
                    children: Box::new(children),
                };
                //new_self.insert_vec(items);
                //items.into_iter().for_each(|item| new_self.insert(item));
                unsafe {
                    std::ptr::write(self, new_self);
                };
                Ok(())
            }
            other => {
                unsafe {
                    std::ptr::write(self, other);
                };
                Err(ErrorKind::NodeAlreadySplit)
            }
        }
    }
}

impl<'a, T, const K: usize> KdTree<'a, T, K>
where
    T: Halveable + Clone,
    Range<T>: MidPoint<T>,
    [(); 1 << K]: Sized,
{
    pub fn new(ranges: impl Into<[Range<T>; K]>, capacity: usize) -> Self {
        let capacity: NonZero<usize> =
            capacity.try_into().expect("Capacity must be nonzero");
        let root = Node::new(ranges.into(), capacity);
        Self {
            root,
            capacity,
            count: 0,
        }
    }

    fn insert_point(&mut self, point: [T; K]) {
        if self
            .root
            .get_ranges()
            .iter()
            .enumerate()
            .all(|(axis, range)| range.contains(&point[axis]))
        {
            self.root.insert(point, self.count);
            self.count += 1;
        }
    }

    pub fn insert<Item>(&mut self, item: Item)
    where
        Item: Into<[T; K]> + ?Sized,
    {
        self.insert_point(item.into());
    }

    pub fn insert_slice<Item>(&mut self, items: &[Item])
    where
        for<'b> &'b Item: Into<[T; K]>,
    {
        for item in items {
            let point: [T; K] = item.into();
            self.insert_point(point);
        }
    }

    pub fn insert_vec(&mut self, vec: Vec<[T; K]>) {
        vec.into_iter().for_each(|p| self.insert_point(p));
    }
}

#[cfg(test)]
mod tests {}
